// ====================================================================
// pan_search.cpp
//
// Pan 소실 탐색 상태기계 (조준 fallback).
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// ================================================================
// Pan 탐색 상태 갱신
//
// 상태 4개.
//
//   Idle      목표 0. 좌/우 흰선 중 한쪽만 pan_trigger_miss_frames_
//             프레임 연속으로 안 잡히면 그 방향으로 Searching 진입.
//
//   Searching 첫 명령은 pan_start_deg_ 로 한 번에 간다 (코너에서
//             몇 도로는 차선이 다시 안 들어온다). 그 뒤로는
//             pan_step_deg_ 씩 전진하되, 매 스텝 실제 각도가
//             정착(settled)할 때까지 기다린다. 정착한 프레임에서
//             해당 쪽이 잡히면 그 각도에 멈추고 Holding.
//             pan_max_deg_ 를 넘게 되면 포기하고 Returning.
//
//   Holding   찾은 각도를 유지한 채, 정면으로 돌아가도 그 선을
//             다시 잡을 수 있게 될 때까지 기다린다. 재검출 즉시
//             복귀하면 그 선을 곧바로 다시 놓치고 pan 이 왕복
//             하면서 그동안 경로가 계속 끊긴다.
//
//             판정은 countVisibleAtZeroPan — 찾은 선의 추적점
//             중 pan = 0 에서도 화면에 남는 것이
//             pan_return_min_points_ 개 이상인 프레임이
//             pan_return_confirm_frames_ 번 연속되면 Returning.
//
//             그때까지 복귀하지 않는다 (기본값). pan_hold_timeout_s_
//             를 양수로 주면 그때만 상한이 생긴다.
//
//   Returning pan_step_deg_ 씩 0 으로 되돌아간다. 도착하면 Idle.
//
// found_count 는 detectLaneSlidingWindow 가 실제로 픽셀을 잡은
// 스텝 수이고, 16-c 복원선은 항상 0 이므로 "이번 프레임에 진짜
// 잡혔는가" 를 그대로 나타낸다.
//
// 노란 중앙선까지 없으면 좌/우 어느 쪽이 사라진 건지 판단할
// 기준이 없으므로 손대지 않는다 (오탐 방지). 둘 다 사라졌을
// 때도 방향을 고를 근거가 없으므로 마찬가지다.
//
// pan != 0 인 프레임의 좌표를 신뢰하지 않는 이유는 이 함수
// 선언부(헤더) 주석 참고 — 동적 BEV 미구현.
// ================================================================

void KauLaneDetectionNode::updatePanSearch(
    const LaneDetectionResult & left,
    const LaneDetectionResult & yellow,
    const LaneDetectionResult & right,
    const LanePath & lane_path,
    const rclcpp::Time & frame_stamp)
{
    // ------------------------------------------------------------
    // Pan 조준 (feedforward) — 아래 탐색 상태기계를 대체한다.
    //
    // 파라미터를 매 프레임 다시 읽는다. bev 파라미터와 같은 idiom이고,
    // pan_search_enable 과 달리 ros2 param set 으로 주행 중에 끄고 켤
    // 수 있다 — 되돌릴 때 재빌드가 필요 없어야 하기 때문이다.
    //
    // 아래 상태기계는 한 줄도 건드리지 않았다. 여기서 조기 반환하지
    // 않으면(기능 꺼짐 / 근거 없음) 예전 동작 그대로다.
    // ------------------------------------------------------------

    pan_aim_enable_ =
        this->get_parameter("pan_aim_enable").as_bool();

    pan_aim_lookahead_m_ =
        this->get_parameter("pan_aim_lookahead_m").as_double();

    pan_aim_gain_ =
        this->get_parameter("pan_aim_gain").as_double();

    pan_aim_source_ =
        this->get_parameter("pan_aim_source").as_string();

    pan_aim_min_evidence_cm_ =
        this->get_parameter("pan_aim_min_evidence_cm").as_double();

    pan_aim_deadband_deg_ =
        this->get_parameter("pan_aim_deadband_deg").as_double();

    pan_aim_hold_frames_ =
        static_cast<int>(
            this->get_parameter("pan_aim_hold_frames").as_int()
        );

    pan_aim_fov_margin_deg_ =
        this->get_parameter("pan_aim_fov_margin_deg").as_double();


    last_pan_aim_valid_ = false;


    if (
        pan_aim_enable_ &&
        pan_search_enable_
    )
    {
        // --------------------------------------------------------
        // 근거 선택
        //
        //   lane    이 노드가 방금 만든 경로 (측위 불필요, 기본)
        //   global  /path/global + 측위 (예전 동작)
        //
        // 아는 값이 아니면 lane 으로 본다 — 오타로 카메라가 통째로
        // 죽는 것보다 기본 동작으로 흐르는 쪽이 낫다.
        // --------------------------------------------------------

        const bool use_global = (pan_aim_source_ == "global");

        if (
            !use_global &&
            pan_aim_source_ != "lane"
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "pan_aim_source 가 '%s' 다 (lane | global). "
                "lane 으로 본다.",
                pan_aim_source_.c_str()
            );
        }


        bool aim_valid = false;

        double aim_deg =
            use_global
                ? panAimGoalDeg(
                      lookupVehiclePose(frame_stamp),
                      &aim_valid
                  )
                : panAimGoalDegFromLane(
                      lane_path,
                      &aim_valid
                  );


        // --------------------------------------------------------
        // 근거가 빈 프레임은 직전 조준각으로 버틴다
        //
        // 여기서 0 으로 되돌리면 안 된다. "안 보임 -> 정면 복귀 ->
        // 보임 -> 다시 조준" 이 §8 실측 왕복(15초 9회)의 형태
        // 그대로다. 점선 공백 한두 장, 가림 한두 장은 홀드로 넘긴다.
        //
        // 한도를 넘기면 조준을 포기하고 아래 탐색 상태기계로 넘어간다
        // — 차선이 통째로 사라진 상황은 "찾아 도는" 쪽이 맡는 게 맞다.
        // --------------------------------------------------------

        bool holding = false;

        if (
            !aim_valid &&
            pan_aim_goal_init_ &&
            pan_aim_hold_streak_ < pan_aim_hold_frames_
        )
        {
            aim_deg = pan_aim_goal_deg_;

            aim_valid = true;

            holding = true;

            ++pan_aim_hold_streak_;
        }


        if (aim_valid)
        {
            if (!holding)
            {
                pan_aim_hold_streak_ = 0;
            }


            // 상한 — pan_max_deg 와 "근거가 화면에 남는 각" 중
            // 작은 쪽. commandPan 이 pan_max_deg 로 한 번 더
            // 자르지만 데드밴드 비교를 자른 값끼리 해야 상한에
            // 붙어 있는 동안 목표가 흔들리지 않는다.
            const double limit_deg = panAimLimitDeg();

            aim_deg =
                std::clamp(
                    aim_deg,
                    -limit_deg,
                    limit_deg
                );


            // 데드밴드 — 목표를 다시 세울 만큼 달라졌을 때만
            // 세운다. 매 프레임 목표가 미세하게 바뀌면 램프가
            // 계속 다시 출발해 "꺾다 말다" 로 보인다. 비교
            // 기준이 직전 조준각이 아니라 **직전 목표각**이라
            // 오차가 누적되지 않는다 (상한 = 데드밴드).
            if (
                !pan_aim_goal_init_ ||
                std::abs(aim_deg - pan_aim_goal_deg_) >=
                    pan_aim_deadband_deg_
            )
            {
                pan_aim_goal_deg_ = aim_deg;

                pan_aim_goal_init_ = true;
            }


            last_pan_aim_deg_ = pan_aim_goal_deg_;

            last_pan_aim_valid_ = true;

            commandPan(pan_aim_goal_deg_);

            // 상태기계를 Idle 로 눌러 둔다. 근거가 끊겨 아래로
            // 흘러가게 될 때 낡은 상태/카운트에서 재개하지 않도록.
            pan_state_ = PanSearchState::Idle;

            pan_search_dir_ = 0;

            left_miss_streak_ = 0;

            right_miss_streak_ = 0;

            pan_straight_streak_ = 0;

            pan_found_streak_ = 0;

            publishPanRamped(frame_stamp);

            return;
        }


        // 조준 포기. 아래 탐색 상태기계로 흘려 보낸다 — 조준이
        // 유일한 근거였다면 이 구간에서 카메라가 통째로 죽는다
        // (§9 가 시야 기반 근거를 따로 둔 이유와 같다).
        //
        // 홀드 상태를 끊어 둔다. 다음에 근거가 돌아오면 낡은
        // 목표각이 아니라 그때의 조준각에서 새로 시작해야 한다.
        pan_aim_hold_streak_ = 0;

        pan_aim_goal_init_ = false;

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "%s. 탐색 상태기계로 대체한다.",
            use_global
                ? "Pan 조준 불가 (측위 또는 /path/global 없음)"
                : "Pan 조준 불가 (차선 근거 부족)"
        );


        // ------------------------------------------------------------
        // 카메라가 이미 돌아가 있으면 Holding 으로 넘긴다.
        //
        // 조준은 매 프레임 pan_state_ 를 Idle 로 눌러 두므로, 여기서
        // 그냥 흘려 보내면 "돌아간 각을 든 채 Idle" 이 된다. 그 상태를
        // 곧바로 0 으로 되돌리면 **직선을 확인하기도 전에 복귀**한다 —
        // 근거가 한두 프레임 끊긴 것뿐인데 커브 한가운데서 정면으로
        // 돌아섰다가 다시 조준하는 왕복이 된다 (§8 실측 15초 9회의
        // 형태 그대로다).
        //
        // 그래서 복귀 판정을 Holding 에 맡긴다. 이미 있는 게이트가
        // 정확히 이 질문에 답한다 — "정면으로 돌려도 그 선이 화면에
        // 남는가" 를 pan_return_confirm_frames 프레임 연속으로 본다.
        // 끝내 안 서면 pan_hold_timeout_s 가 상한을 준다.
        //
        // pan_search_dir_ 은 Holding 이 "어느 쪽 선을 보고 판정할지"
        // 고르는 데 쓰므로 여기서 세워야 한다. 조준 경로는 이 값을
        // 0 으로 두는데, 그대로 두면 Holding 이 부호 판정에서 항상
        // 오른쪽을 보게 된다. 지금 돌아가 있는 방향이 곧 보고 있는
        // 방향이다 (+ = 왼쪽).
        // ------------------------------------------------------------
        if (
            pan_search_enable_ &&
            std::abs(pan_cmd_deg_) > 1e-6 &&
            pan_state_ == PanSearchState::Idle
        )
        {
            pan_search_dir_ = (pan_cmd_deg_ > 0.0) ? +1 : -1;

            pan_state_ = PanSearchState::Holding;

            pan_straight_streak_ = 0;

            pan_hold_start_ = frame_stamp;

            RCLCPP_INFO(
                this->get_logger(),
                "조준 포기 시점에 카메라가 %+.1fdeg 돌아가 있다. "
                "바로 복귀하지 않고 유지하며 복귀 조건을 본다 "
                "(정면에서도 %s 흰선이 %d점 이상 %d프레임 연속 "
                "보이면 복귀, 최대 %.1fs).",
                pan_cmd_deg_,
                (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                std::max(4, pan_return_min_points_),
                pan_return_confirm_frames_,
                pan_hold_timeout_s_
            );
        }
    }


    if (!pan_search_enable_)
    {
        // 기능이 꺼지는 순간에도 카메라가 돌아간 채로 남지
        // 않도록 원위치를 보장한다.
        if (
            pan_state_ != PanSearchState::Idle ||
            std::abs(pan_cmd_deg_) > 1e-6
        )
        {
            commandPan(0.0);

            pan_state_ = PanSearchState::Idle;

            pan_search_dir_ = 0;
        }

        left_miss_streak_ = 0;

        right_miss_streak_ = 0;

        // 꺼지는 중에도 원위치까지는 램프로 돌아가야 한다.
        if (std::abs(pan_cmd_deg_ - pan_goal_deg_) > 1e-9)
        {
            publishPanRamped(frame_stamp);
        }

        return;
    }


    // 다음 스텝으로 넘어가도 되는가 — 두 가지를 **둘 다** 본다.
    //
    //  (1) 램프가 목표에 닿았는가 (at_goal)
    //  (2) 서보가 그 명령에 도달했는가 (servo_ok)
    //
    // 예전에는 (2) 만 봤다. 그런데 실기 드라이버는 서보 인코더가
    // 없어 마지막 명령을 그대로 되쏘므로 (2) 는 **항상 참**이다.
    // 즉 실기에서는 정착 대기가 사실상 없었고, 스텝이 프레임마다
    // 나가 서보가 끝까지 쫓기지 못한 채 각도만 앞서 갔다.
    // (1) 이 그 공백을 메운다 — 램프는 우리가 발행하는 값이라
    // 인코더 없이도 정직하게 여러 프레임이 걸린다.
    const bool at_goal =
        std::abs(pan_cmd_deg_ - pan_goal_deg_) <= 1e-9;

    const bool servo_ok =
        !joint_state_received_ ||
        std::abs(actual_pan_deg_ - pan_cmd_deg_) <=
            pan_settle_tol_deg_;

    const bool settled = at_goal && servo_ok;


    switch (pan_state_)
    {
        case PanSearchState::Idle:
        {
            // ------------------------------------------------
            // 안전망 — Idle 인데 목표각이 0 이 아닌 경우.
            //
            // Idle 은 "카메라가 돌아 있을 이유가 없는 상태" 이므로
            // 목표각도 0 이어야 하는데, 그걸 보장하는 주체가
            // 없었다. 아래 분기들은 **탐색을 새로 걸 때만**
            // commandPan 을 부르고 나머지 경로는 전부 그냥
            // break 라, 어디선가 물려받은 각이 매 프레임 그대로
            // 재발행됐다 -- "돌아간 뒤 복귀를 안 함" 의 정체다.
            //
            // 정상 경로에서는 여기가 안 돈다:
            //
            //   Returning 완료 -> 목표각이 이미 0
            //   조준 포기      -> 위에서 Holding 으로 넘긴다
            //                     (복귀 판정을 거쳐서 돌아온다)
            //
            // 즉 여기 걸린다는 것은 위 두 경로를 타지 않고 각이
            // 남았다는 뜻이다. 그때만 램프로 0 으로 되돌린다.
            // 이번 프레임에 탐색이 걸리면 아래
            // commandPan(pan_start_deg_ * dir) 이 덮으므로
            // 순서상 손해가 없다.
            // ------------------------------------------------
            if (std::abs(pan_goal_deg_) > 1e-9)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Pan 복귀(안전망): Idle 인데 목표각이 남아 있다 "
                    "(%+.1fdeg -> 0.0deg, %.0fdeg/s 램프, 약 %.2fs)",
                    pan_goal_deg_,
                    pan_rate_deg_s_,
                    (pan_rate_deg_s_ > 0.0)
                        ? std::abs(pan_goal_deg_) / pan_rate_deg_s_
                        : 0.0
                );

                commandPan(0.0);
            }


            left_miss_streak_ =
                (left.found_count == 0)
                    ? left_miss_streak_ + 1 : 0;

            right_miss_streak_ =
                (right.found_count == 0)
                    ? right_miss_streak_ + 1 : 0;


            if (yellow.found_count == 0)
            {
                break;
            }


            const bool left_missing =
                left_miss_streak_ >= pan_trigger_miss_frames_;

            const bool right_missing =
                right_miss_streak_ >= pan_trigger_miss_frames_;


            if (left_missing == right_missing)
            {
                // 둘 다 정상이거나 둘 다 소실 -> 방향을 고를 근거 없음.
                break;
            }


            // pan_search_dir_ 은 회전 방향이다 (+1 왼쪽 / -1 오른쪽).
            //
            // URDF physicar.urdf.xacro 의 camera_pan_joint 와
            // 시뮬 model.sdf 의 같은 관절 모두 axis = +Z 이므로
            // +rad 이 왼쪽(반시계)이다. 따라서 왼쪽 흰선을
            // 놓쳤으면 +, 오른쪽이면 - 로 돌려야 소실된 쪽을 본다.
            //
            // (예전에는 부호가 반대였다. 실기에서 놓친 쪽의
            //  반대편으로 고개를 돌리는 것을 확인하고 고쳤다.
            //  pan_sign_ 은 이것과 무관한 하드웨어 배선용
            //  뒤집개이므로 그쪽으로 덮지 말 것.)
            pan_search_dir_ = left_missing ? +1 : -1;


            // 곡률 룩어헤드 + 장애물 (선언부 헤더 주석, 클래스
            // 상단 "Pan 트리거 판정" 주석 참고). 자차 map 위치를
            // 한 번만 조회해 둘 다에 쓴다.
            //
            // 곡률은 두 근거를 OR 로 합친다.
            //
            //   경로 기반   /path/global 위 자차 전방 룩어헤드
            //               최대곡률. 측위(map<-base_link TF)와
            //               /path/global 수신이 둘 다 있어야 한다.
            //   시야 기반   가려지지 않은 쪽 — 즉 이 트리거를 아직
            //               통과 못 시킨 노란 중앙선(yellow, 이
            //               Idle 분기는 yellow.found_count>0 을
            //               이미 전제한다)의 꺾임(trackBendDeg).
            //               측위/전역경로 없이도 항상 계산된다.
            //
            // 가려진 쪽(놓친 흰선)의 곡률을 쓰면 안 된다 — 애초에
            // 안 보이는 선이라 잴 수 없고, 억지로 재도 장애물
            // 자체의 윤곽을 곡률로 오인하게 된다. 반드시 가려지지
            // 않은 쪽으로 판정한다.
            //
            // 경로 기반이 아직 무근거(-1, 측위 기동 전 등)일 때
            // 시야 기반 없이 OR 를 빼면, 장애물 판정마저 근거가
            // 없는 프레임(측위/객체인식 미기동)에서 직선 구간의
            // 가려진 흰선에도 그냥 탐색이 걸린다 — 시야 기반이
            // 그 공백을 메운다.
            //
            // 곡률 우선: 둘 중 하나라도 급커브로 판정되면 그 방향에
            // 장애물 원이 걸려도(트랙 경계벽 오검출 가능성) 무시
            // 하고 탐색을 진행한다. 급커브가 아닐 때만 장애물
            // 판정으로 넘어간다.
            //
            // 가려서 안 보이는 것이면 고개를 돌려도 소용없다.
            // 가린 물체가 같이 따라오기 때문이다. 상한까지
            // 헛돌면서 경로만 끊길 뿐이므로 탐색을 걸지 않는다.
            //
            // 소실 연속 카운트는 리셋하지 않는다. 장애물이
            // 지나가고 나서도 여전히 안 보이면 그때 곧바로
            // 탐색이 걸려야 한다.
            {
                const VehiclePose vehicle_pose =
                    lookupVehiclePose(frame_stamp);

                const double kappa_ahead =
                    curvatureAheadKappa(vehicle_pose);

                const bool curve_ahead_path =
                    kappa_ahead >= curve_ahead_kappa_thresh_;

                // yellow 는 이 Idle 분기에 들어온 시점에 이미
                // found_count > 0 이 보장돼 있다.
                const double yellow_bend_deg =
                    geom::trackBendDeg(yellow.track_px);

                const bool curve_ahead_bend =
                    yellow_bend_deg >= curve_ahead_bend_deg_thresh_;

                const bool curve_ahead =
                    curve_ahead_path || curve_ahead_bend;

                if (!curve_ahead)
                {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        3000,
                        "%s 흰선이 안 보이지만 전방이 커브가 아니다 "
                        "(경로곡률 %.4f 1/cm < %.4f, 중앙선꺾임 %.1fdeg < %.1fdeg). "
                        "가림으로 보고 pan 하지 않는다.",
                        (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                        (kappa_ahead >= 0.0) ? kappa_ahead : 0.0,
                        curve_ahead_kappa_thresh_,
                        yellow_bend_deg,
                        curve_ahead_bend_deg_thresh_
                    );

                    pan_search_dir_ = 0;

                    break;
                }
            }


            pan_state_ = PanSearchState::Searching;

            pan_found_streak_ = 0;

            // 첫 목표는 쓸 만한 각도로 크게 잡되, 도달은 램프로 한다.
            commandPan(pan_start_deg_ * pan_search_dir_);

            RCLCPP_INFO(
                this->get_logger(),
                "Pan 탐색 시작: %s 흰선 %d프레임 연속 소실 "
                "-> 목표 %+.1fdeg (%.0fdeg/s 램프, 약 %.2fs)",
                (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                pan_trigger_miss_frames_,
                pan_goal_deg_,
                pan_rate_deg_s_,
                (pan_rate_deg_s_ > 0.0)
                    ? std::abs(pan_goal_deg_) / pan_rate_deg_s_ : 0.0
            );

            break;
        }

        case PanSearchState::Searching:
        {
            // 카메라가 명령을 따라잡기 전 영상으로 판정하면
            // 아직 도착하지 않은 각도의 시야를 본 것이 된다.
            if (!settled)
            {
                break;
            }


            // 창 수 + 연속 프레임. 왜 창 하나로는 안 되는지는
            // 선언부 pan_found_min_windows_ 주석 참고.
            const int side_windows =
                (pan_search_dir_ > 0)
                    ? left.found_count
                    : right.found_count;

            pan_found_streak_ =
                (side_windows >= pan_found_min_windows_)
                    ? pan_found_streak_ + 1
                    : 0;

            const bool found =
                pan_found_streak_ >= pan_found_confirm_frames_;


            if (found)
            {
                const double at_deg = pan_cmd_deg_;

                // 기본값 0 이면 찾은 그 각도에 그대로 선다.
                if (pan_overshoot_deg_ > 0.0)
                {
                    commandPan(
                        pan_cmd_deg_ +
                        pan_overshoot_deg_ * pan_search_dir_
                    );
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Pan 탐색 성공: %+.1fdeg 에서 재검출 "
                    "(창 %d개 >= %d, %d프레임 연속). "
                    "%+.1fdeg 에 정지하고 직선 구간까지 유지.",
                    at_deg,
                    side_windows,
                    pan_found_min_windows_,
                    pan_found_confirm_frames_,
                    pan_cmd_deg_
                );

                pan_state_ = PanSearchState::Holding;

                pan_straight_streak_ = 0;

                pan_hold_start_ = frame_stamp;

                break;
            }


            const double next =
                pan_cmd_deg_ + pan_step_deg_ * pan_search_dir_;


            if (std::abs(next) > pan_max_deg_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Pan 탐색 실패: %.1fdeg 까지 돌렸지만 "
                    "재검출 못 함 (마지막 창 %d개, 기준 %d). "
                    "포기하고 복귀.",
                    pan_max_deg_,
                    side_windows,
                    pan_found_min_windows_
                );

                pan_state_ = PanSearchState::Returning;

                break;
            }


            commandPan(next);

            break;
        }

        case PanSearchState::Holding:
        {
            // 복귀 조건: 정면으로 돌아가도 그 선이 보이는가.
            //
            // 예전에는 "도로가 펴졌는가" 를 봤는데, 그건 다른
            // 명제다. 곧은 구간에서도 그 선이 화면 밖에 있으면
            // 복귀 -> 즉시 재탐색으로 왕복한다 (실측 15초에 9회,
            // countVisibleAtZeroPan 선언부 주석 참고).
            //
            // 그래서 목적을 직접 잰다 — 찾은 선의 추적점 중
            // pan = 0 에서도 화면에 남는 것이 검출에 충분한
            // 개수인가.
            const int min_pts =
                std::max(4, pan_return_min_points_);

            const LaneDetectionResult & side =
                (pan_search_dir_ > 0) ? left : right;


            // 꺾임각은 이제 판정에 쓰지 않는다. status 의 pbend
            // 로만 내보내 튜닝 참고용으로 남긴다.
            last_bend_deg_ =
                (side.found_count > 0)
                    ? geom::trackBendDeg(side.track_px)
                    : -1.0;


            const int visible =
                (side.found_count > 0)
                    ? countVisibleAtZeroPan(
                          side.track_px,
                          pan_cmd_deg_)
                    : -1;

            last_visible_at_zero_ = visible;


            if (visible < 0)
            {
                // 근거 자체가 없다 (그 선을 이번 프레임에 못 봤거나
                // 캘리브레이션 전). 판정 불가 -> 카운트를 끊는다.
                //
                // ★ 예전에는 여기서 break 했다. 그런데 아래
                //   pan_hold_timeout_s 검사가 이 break 뒤에 있어서,
                //   **근거가 아예 없는 동안에는 타임아웃이 한 번도
                //   돌지 않았다.** 조준 포기로 Holding 에 들어오는
                //   경우가 정확히 그 상황이라(근거가 없어서 포기한
                //   것이다) 상한이 있으나 마나였다. break 를 빼고
                //   아래 타임아웃까지 흘려 보낸다 — 복귀 판정은
                //   streak 가 0 이라 어차피 통과하지 못한다.
                pan_straight_streak_ = 0;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "복귀 판정 보류: %s 흰선 근거 없음. %+.1fdeg 유지.",
                    (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                    pan_cmd_deg_
                );
            }
            else if (visible >= min_pts)
            {
                ++pan_straight_streak_;
            }
            else
            {
                pan_straight_streak_ = 0;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "복귀 판정 보류: 정면으로 돌아가면 %s 흰선 "
                    "%d점 중 %d점만 화면에 남는다 (%d점 필요). "
                    "%+.1fdeg 유지.",
                    (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                    side.found_count,
                    visible,
                    min_pts,
                    pan_cmd_deg_
                );
            }


            if (
                visible >= 0 &&
                pan_straight_streak_ >= pan_return_confirm_frames_
            )
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "복귀 가능: 정면에서도 %s 흰선 %d점이 화면에 "
                    "남는다 (%d프레임 연속). %+.1fdeg 에서 복귀 시작.",
                    (pan_search_dir_ > 0) ? "왼쪽" : "오른쪽",
                    visible,
                    pan_return_confirm_frames_,
                    pan_cmd_deg_
                );

                pan_state_ = PanSearchState::Returning;

                break;
            }


            // 0 이하 = 무한 대기. 직선을 만날 때까지 복귀하지 않는다.
            if (pan_hold_timeout_s_ <= 0.0)
            {
                break;
            }


            const double held =
                (frame_stamp - pan_hold_start_).seconds();

            // 기동 직후 node clock 이 넘어가면 held 가 튄다.
            if (held < 0.0)
            {
                pan_hold_start_ = frame_stamp;

                break;
            }

            if (held > pan_hold_timeout_s_)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "%.1fs 동안 직선 구간을 못 만났다. "
                    "%+.1fdeg 에서 그냥 복귀한다.",
                    held,
                    pan_cmd_deg_
                );

                pan_state_ = PanSearchState::Returning;
            }

            break;
        }

        case PanSearchState::Returning:
        {
            // 목표를 0 으로 한 번만 세우고, 나머지는 램프가 한다.
            // 예전에는 pan_step_deg_ 씩 계단으로 되돌아왔다.
            if (std::abs(pan_goal_deg_) > 1e-9)
            {
                commandPan(0.0);

                break;
            }


            if (!settled)
            {
                break;
            }


            pan_state_ = PanSearchState::Idle;

            pan_search_dir_ = 0;

            left_miss_streak_ = 0;

            right_miss_streak_ = 0;

            pan_straight_streak_ = 0;

            break;
        }
    }


    // ------------------------------------------------------------
    // 램프 한 프레임 진행.
    //
    // 상태기계 뒤에 두는 이유: 이번 프레임에 목표가 새로 세워졌으면
    // 그 즉시 첫 스텝이 나가고, 목표가 안 바뀐 프레임에도 램프가
    // 계속 진행된다. 목표에 이미 닿아 있으면 같은 값을 재발행하는
    // 것뿐이라 (절대각 명령이므로) 무해하다.
    // ------------------------------------------------------------

    publishPanRamped(frame_stamp);
}


// ================================================================
// pan = 0 으로 되돌렸을 때 화면에 남는 점 수
//
// 설계 근거와 기하는 선언부(헤더) 주석 참고.
// ================================================================

int KauLaneDetectionNode::countVisibleAtZeroPan(
    const std::vector<cv::Point2d> & track_px,
    double pan_deg) const
{
    if (
        track_px.empty() ||
        !camera_calibrated_ ||
        perspective_matrix_.empty()
    )
    {
        return -1;
    }


    const double fx = camera_matrix_.at<double>(0, 0);

    const double cx = camera_matrix_.at<double>(0, 2);

    if (
        !std::isfinite(fx) ||
        fx <= 1e-6
    )
    {
        return -1;
    }


    // BEV -> 원본 영상. 순수 2D 사상이라 pan 과 무관하게 정확하다.
    cv::Mat inv;

    if (!cv::invert(perspective_matrix_, inv))
    {
        return -1;
    }


    std::vector<cv::Point2f> bev;

    bev.reserve(track_px.size());

    for (const cv::Point2d & p : track_px)
    {
        bev.emplace_back(
            static_cast<float>(p.x),
            static_cast<float>(p.y)
        );
    }


    std::vector<cv::Point2f> img;

    cv::perspectiveTransform(bev, img, inv);


    // 화면 반각. cx 가 곧 화면 중심이므로 그대로 쓴다.
    const double half_fov = std::atan(cx / fx);

    const double limit =
        half_fov -
        pan_return_fov_margin_deg_ * M_PI / 180.0;

    if (limit <= 0.0)
    {
        return -1;
    }


    const double theta = pan_deg * M_PI / 180.0;


    int visible = 0;

    for (const cv::Point2f & q : img)
    {
        // 영상각 (오른쪽 +)
        const double a =
            std::atan(
                (static_cast<double>(q.x) - cx) / fx
            );

        // pan = 0 일 때의 영상각
        const double a0 = a - theta;

        if (std::abs(a0) <= limit)
        {
            ++visible;
        }
    }


    return visible;
}
