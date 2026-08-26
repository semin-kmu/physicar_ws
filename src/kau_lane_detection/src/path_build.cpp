// ====================================================================
// path_build.cpp
//
// 추적점 -> quintic Bezier 제어점. EMA 강건화와 발행 게이트까지.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// path_ema_ctrl_ 의 각 제어점을 target 방향으로 이번 프레임에
// max_step_cm 이하로만 이동시킨다. 정상 블렌드든 재잠금 스냅이든
// 이걸 거치면 "얼마나 다른 값이 왔는가"와 무관하게 프레임당 물리적
// 이동량 상한이 걸린다.
void KauLaneDetectionNode::stepEmaCtrlToward(
    const kau::bezier::Ctrl & target,
    double max_step_cm)
{
    // 점마다 다른 비율을 쓰면 제어점 사이 상대 위치가 뒤틀려
    // 곡선 모양이 매 프레임 달라진다(cusp/스파이크 원인). 전체
    // 제어점에 "같은" 비율을 적용해야 두 곡선의 아핀 블렌드로
    // 남아 모양이 보존된다.
    double max_dist = 0.0;
    for (std::size_t i = 0; i < path_ema_ctrl_.size(); ++i)
    {
        max_dist = std::max(
            max_dist,
            std::hypot(
                target[i].x - path_ema_ctrl_[i].x,
                target[i].y - path_ema_ctrl_[i].y
            )
        );
    }
    const double s =
        (max_dist <= max_step_cm || max_dist < 1e-9)
            ? 1.0
            : max_step_cm / max_dist;
    for (std::size_t i = 0; i < path_ema_ctrl_.size(); ++i)
    {
        path_ema_ctrl_[i].x += s * (target[i].x - path_ema_ctrl_[i].x);
        path_ema_ctrl_[i].y += s * (target[i].y - path_ema_ctrl_[i].y);
    }
}


// ================================================================
// 제어점 열 사이의 평균 거리 [cm]
// ================================================================

double KauLaneDetectionNode::pathDeviationCm(
    const kau::bezier::Ctrl & a,
    const kau::bezier::Ctrl & b)
{
    if (
        a.size() != b.size() ||
        a.empty()
    )
    {
        return -1.0;
    }


    double sum = 0.0;

    for (
        std::size_t i = 0;
        i < a.size();
        ++i
    )
    {
        sum +=
            std::hypot(
                a[i].x - b[i].x,
                a[i].y - b[i].y
            );
    }


    return sum / static_cast<double>(a.size());
}


// ================================================================
// ctrl 이 바뀐 뒤 진단값 재계산
//
// buildCenterlinePath 8장과 같은 식이다. EMA 로 섞은 제어점은
// 관측 당시의 길이/곡률/cte 와 짝이 맞지 않으므로 다시 잰다.
// valid_length 는 여기서 손대지 않는다 (관측이 어디까지
// 근거였는지는 기하가 아니라 관측의 성질이라, 부르는 쪽이
// 따로 EMA 한다).
// ================================================================

void KauLaneDetectionNode::refreshPathMetrics(LanePath & path)
{
    path.length_cm =
        kau::bezier::segLength(path.ctrl);

    path.kappa_max =
        kau::bezier::kappaMaxExact(path.ctrl);


    const kau::bezier::Nearest near =
        kau::bezier::nearestOnSeg(
            path.ctrl,
            kau::bezier::Point2{0.0, 0.0}
        );

    const kau::bezier::Point2 foot =
        kau::bezier::evalSeg(path.ctrl, near.u);

    const double th =
        kau::bezier::heading(path.ctrl, near.u);


    // 좌측 +
    path.cte_cm =
        -std::sin(th) * (0.0 - foot.x) +
         std::cos(th) * (0.0 - foot.y);

    path.heading_err =
        std::remainder(-th, 2.0 * M_PI);
}


void KauLaneDetectionNode::robustifyPath(
    LanePath & path,
    const rclcpp::Time & frame_stamp)
{
    if (!path_ema_enable_) return;

    if (!path.valid) { /* 기존 그대로 */ return; }

    ema_miss_streak_ = 0;

    if (!path_ema_valid_)
    {
        // 첫 관측은 캡 없이 그대로 (움직일 이전 값이 없음)
        path_ema_ctrl_ = path.ctrl;
        conf_ema_ = path.confidence;
        valid_len_ema_ = path.valid_length_cm;
        path_ema_valid_ = true;
        path_ema_reject_streak_ = 0;
        path_ema_time_init_ = false;
        return;
    }

    // ------------------------------------------------------------
    // dt — 게이트와 이동 상한이 **둘 다** 이걸 써야 한다.
    //
    // 예전에는 dt 를 아래 이동 상한 직전에만 구했고, 이상치 게이트는
    // dt 와 무관한 절대값(path_ema_gate_cm_)이었다. 두 안전장치의
    // 단위가 어긋나 있었던 것이다:
    //
    //   이동 상한  path_max_step_cm_per_s * dt   [속도]  -> 자동 조정
    //   이상치 게이트  path_ema_gate_cm            [절대]  -> 고정
    //
    // 14 Hz 에서 15 cm/프레임은 210 cm/s 에 해당한다. 그런데 4 Hz 로
    // 떨어지면 같은 15 cm 가 60 cm/s 가 되어 **3.5배 더 조여진다.**
    // 그러면 프레임이 느려질수록 멀쩡한 관측이 전부 이상치로 걷어
    // 차이고 추정치가 얼어붙는다. 얼어붙은 추정치는 base_link 기준
    // 고정이라, 차가 움직여도 그 자리에 머물러 화면에서는 "경로가
    // 천천히 원래 자리로 돌아오는" 것으로 보인다.
    //
    // 실측 근거: 부하가 높아 프레임이 느려진 구간의 로그 60건에서
    // 편차 중앙 30.4 cm / 최대 76.8 cm 였다 (상한 15.0). 재잠금이
    // 105 회 걸렸다.
    // ------------------------------------------------------------

    double dt = kNominalFrameSec;

    if (path_ema_time_init_)
    {
        dt = (frame_stamp - path_ema_last_stamp_).seconds();

        if (dt < 0.0 || dt > 1.0)
        {
            dt = kNominalFrameSec;
        }
    }

    path_ema_last_stamp_ = frame_stamp;

    path_ema_time_init_ = true;


    const double dev = pathDeviationCm(path.ctrl, path_ema_ctrl_);
    last_path_dev_cm_ = dev;


    // 공칭 주기(14 Hz)에서는 설정값 그대로다 — 기존 동작 보존.
    // 느려진 만큼만 비례해 넓히고, 상한을 둬서 파이프라인이 멎었을
    // 때 게이트가 통째로 무력화되는 것은 막는다.
    const double gate_scale =
        std::clamp(
            dt / kNominalFrameSec,
            1.0,
            path_ema_gate_dt_scale_max_
        );

    const double gate_cm = path_ema_gate_cm_ * gate_scale;

    last_path_gate_cm_ = gate_cm;

    const bool outlier =
        (path_ema_gate_cm_ > 0.0) && (dev > gate_cm);

    kau::bezier::Ctrl intended = path_ema_ctrl_;   // 기본값: 변화 없음

    if (outlier)
    {
        ++path_ema_reject_streak_;

        if (path_ema_reject_streak_ >= path_ema_relock_frames_)
        {
            intended = path.ctrl;   // 재잠금 목표 (아래에서 캡 적용됨)
            conf_ema_ = path.confidence;
            valid_len_ema_ = path.valid_length_cm;
            path_ema_reject_streak_ = 0;

            RCLCPP_INFO(this->get_logger(),
                "새 경로 주장이 %d 프레임 연속 (평균 %.1fcm 차이). "
                "재잠금 목표를 세운다 (실제 이동은 %.0fcm/s로 캡).",
                path_ema_relock_frames_, dev, path_max_step_cm_per_s_);
        }
        else
        {
            conf_ema_ *= (1.0 - path_ema_alpha_);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "관측이 추정치에서 평균 %.1fcm 벗어났다 (상한 %.1f, "
                "dt 보정 후). "
                "섞지 않고 신뢰도만 %.2f 로 내린다.",
                dev, gate_cm, conf_ema_);
        }
    }
    else
    {
        path_ema_reject_streak_ = 0;

        const double a =
            std::clamp(path_ema_alpha_ * path.confidence, 0.0, 1.0);

        for (std::size_t i = 0; i < path_ema_ctrl_.size(); ++i)
        {
            intended[i].x =
                path_ema_ctrl_[i].x + a * (path.ctrl[i].x - path_ema_ctrl_[i].x);
            intended[i].y =
                path_ema_ctrl_[i].y + a * (path.ctrl[i].y - path_ema_ctrl_[i].y);
        }

        conf_ema_ += path_ema_alpha_ * (path.confidence - conf_ema_);
        valid_len_ema_ += path_ema_alpha_ * (path.valid_length_cm - valid_len_ema_);
    }

    // --- 실제 이동은 여기서 한 번에 캡 적용 (dt 는 위에서 구했다) ---
    stepEmaCtrlToward(intended, path_max_step_cm_per_s_ * dt);

    if (!kau::bezier::isRegular(path_ema_ctrl_))
    {
        // 뒤틀린 프레임은 이번엔 갱신하지 않고 직전 값을 그대로 낸다
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "EMA 갱신 결과가 degenerate 하다. 이번 프레임은 직전 추정치를 유지.");
        // path_ema_ctrl_ 를 되돌리거나, 최소한 발행은 건너뛰는 처리 필요
    }

    path.ctrl = path_ema_ctrl_;
    path.confidence = conf_ema_;
    refreshPathMetrics(path);
    path.valid_length_cm = std::min(valid_len_ema_, path.length_cm);
}


int KauLaneDetectionNode::pathGate(
    const LanePath & path,
    double sx,
    double camera_yaw_deg) const
{
    if (!path.built)
    {
        return 1;
    }


    // ------------------------------------------------------------
    // 1. 근거량
    // ------------------------------------------------------------

    if (path.source_windows < min_path_windows_)
    {
        return 1;
    }


    // ------------------------------------------------------------
    // 2. 조향 한계
    //
    //   R_min = wheelbase / tan(max_steer)
    //   18cm / tan(20도) = 49.5cm
    // ------------------------------------------------------------

    // 조향 한계. 기본은 끔 — 최소 회전반경보다 급해도 경로는 그대로
    // 낸다. 곡률은 status 의 rad 로 계속 나가므로 하류가 보고 정한다.
    const double steer_rad =
        max_steer_deg_ * M_PI / 180.0;

    if (
        path_gate_steer_enable_ &&
        wheelbase_cm_ > 0.0 &&
        steer_rad > 1e-6 &&
        path.kappa_max > 1e-9
    )
    {
        const double r_min =
            wheelbase_cm_ / std::tan(steer_rad);

        if (1.0 / path.kappa_max < r_min)
        {
            return 2;
        }
    }


    // ------------------------------------------------------------
    // 3. 시야 이탈
    //
    // BEV 영상 밖으로 나가는 경로는 관측 근거가 없다. 이 한계는
    // "카메라가 지금 보는 방향" 기준의 물리적 화각이므로, 검사
    // 전에 buildCenterlinePath 6-b 절의 회전 보정을 역으로 되돌려
    // 카메라 기준 좌표로 판정한다. base_link(회전 후) 좌표를
    // 그대로 쓰면, pan 으로 실제로는 화각 안인 점이 회전 때문에
    // 횡거리만 커 보여 오탈락할 수 있다.
    // ------------------------------------------------------------

    double limit = max_lateral_cm_;

    if (limit <= 0.0)
    {
        limit = 0.5 * bev_out_width_ * sx;
    }


    const double yaw_rad = -camera_yaw_deg * M_PI / 180.0;

    const double cos_yaw = std::cos(yaw_rad);

    const double sin_yaw = std::sin(yaw_rad);


    for (const kau::bezier::Point2 & c : path.ctrl)
    {
        const double x_rel = c.x - pan_pivot_offset_x_cm_;

        const double y_rel = c.y - pan_pivot_offset_y_cm_;

        const double y_cam =
            pan_pivot_offset_y_cm_ +
            x_rel * sin_yaw + y_rel * cos_yaw;

        if (std::abs(y_cam - path_y_offset_cm_) > limit)
        {
            return 3;
        }
    }


    return 0;
}


// ================================================================
// 발행 게이트
//
// 세 가지를 본다. 전부 "근거 없는 단언을 막는다" 는 한 가지 목적이다.
//
//   1 근거부족  중심선이 딛고 선 실측 창 수가 하한 미만
//   2 조향한계  곡률이 차량 최소 회전반경보다 급함 (실행 불가능)
//   3 시야이탈  제어점이 BEV 영상 밖. 본 적 없는 곳을 단언하는 것
//
// 3번은 Bezier convex hull 성질을 쓴다. 곡선은 제어점의 볼록껍질
// 안에 있으므로 제어점만 검사하면 곡선 전체가 보장된다.
// 표본 추출이 아니라 증명이다.
// ================================================================

KauLaneDetectionNode::GateLabel KauLaneDetectionNode::gateLabel(int code)
{
    switch (code)
    {
        case 1:  return { "근거부족", "THIN"  };
        case 2:  return { "조향한계", "STEER" };
        case 3:  return { "시야이탈", "OUT"   };
        default: return { "ok",       "ok"    };
    }
}


// ================================================================
// 차선 중심선 -> quintic Bezier 제어점
//
// 중심선 후보는 세 차선이 각각 하나씩 내놓는다.
//
//   왼쪽 흰선   법선 방향 + lane_width_px
//   노란 중앙선 그대로
//   오른쪽 흰선 법선 방향 - lane_width_px
//
// offset 이 x 가 아니라 "국소 법선" 인 것이 중요하다.
// 90도 코너에서 차선이 가로로 누우면 중심선은 x 가 아니라
// y 로 밀려야 한다 (offsetTrack 참고).
//
// 세 후보를 하나의 점구름으로 합쳐서, 누적 현길이 s 를
// 매개변수로 x(s), y(s) 를 가중 최소제곱 적합한다.
// 가중치는 "실제로 픽셀을 잡은 스텝 수(found_count)".
// 반대편에서 복원된 차선은 found_count 가 0 이라 자동으로
// 가중치 0 이 된다. 복원선은 새 정보가 없으니 맞다.
//
// 세 차선은 서로 평행하고 모두 BEV 하단에서 출발하므로
// 각자의 s 가 서로 비교 가능하다.
//
// 이 함수는 자차가 어느 차로에 있는지 묻지 않는다. 세 후보
// 전부를 "중앙선의 위치 추정치"로 환산해 가중합할 뿐이라,
// 결과는 항상 중앙선(황색 점선)의 추정 경로다.
//
// 내부 적합 차수 (docs/경로_형식.md 3항):
//
//   점 6개 이상 -> 3차, 4개 이상 -> 2차, 그 외 -> 1차.
//   어느 쪽이든 degree elevation 으로 5차로 무손실 승격한다.
//
//   90도 코너는 s 에 대한 사분원이라 2차로는 부족하다.
//   3차면 사분원을 충분히 담는다. (예전 코드는 x=f(y) 2차라
//   코너를 아예 표현조차 못 했다.)
// ================================================================

// ================================================================
// 차선 한 줄 -> quintic Bezier 경로  (/lane/left, /lane/right)
//
// buildCenterlinePath 의 3-b ~ 8 절과 **같은 식**이다. 다른 것은
// 입력이 추적점 하나라는 것뿐이라 1~3 절(후보 합성 / spine 선정 /
// 이탈 제거 / s 통일)이 통째로 필요 없다 — 합칠 후보가 없으므로
// 그 점열 자체가 곧 spine 이고 s 는 그 위 누적 호길이다.
//
// 중심선 경로는 이 함수를 거치지 않는다. buildCenterlinePath 는
// 한 줄도 안 건드렸으므로 /lane/center 결과는 그대로다.
// ================================================================

KauLaneDetectionNode::LanePath KauLaneDetectionNode::buildEdgePath(
    const LaneDetectionResult & lane,
    int bev_width,
    int bev_height,
    double camera_yaw_deg)
{
    LanePath path;

    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;

    if (!bevScale(sx, sy, x_base))
    {
        return path;
    }


    // 2차 적합에 최소 4점이 필요하다 (buildCenterlinePath 4절의
    // order 결정과 같은 기준).
    if (
        !lane.valid ||
        lane.track_px.size() < 4
    )
    {
        return path;
    }

    const std::vector<cv::Point2d> & pts = lane.track_px;


    // ------------------------------------------------------------
    // 누적 호길이 (buildCenterlinePath 2절)
    // ------------------------------------------------------------

    std::vector<double> ss(pts.size(), 0.0);

    std::vector<double> xs(pts.size());

    std::vector<double> ys(pts.size());

    std::vector<double> ws(pts.size(), 1.0);

    for (
        std::size_t i = 0;
        i < pts.size();
        ++i
    )
    {
        if (i > 0)
        {
            ss[i] =
                ss[i - 1] +
                cv::norm(pts[i] - pts[i - 1]);
        }

        xs[i] = pts[i].x;

        ys[i] = pts[i].y;
    }

    const double s_max = ss.back();

    if (!(s_max > 1e-6))
    {
        return path;
    }


    // ------------------------------------------------------------
    // 3-b. 실관측 구간 — 점열이 끊긴 지점까지만 근거로 인정한다
    // ------------------------------------------------------------

    double cover_ratio = 1.0;

    {
        const double gap_limit =
            2.0 * std::max(4.0, track_step_px_);

        double s_cover = s_max;

        for (
            std::size_t k = 1;
            k < ss.size();
            ++k
        )
        {
            if (ss[k] - ss[k - 1] > gap_limit)
            {
                s_cover = ss[k - 1];

                break;
            }
        }

        cover_ratio =
            std::clamp(s_cover / s_max, 0.0, 1.0);
    }


    // ------------------------------------------------------------
    // 4. 내부 적합 x(s), y(s)
    // ------------------------------------------------------------

    const int npts =
        static_cast<int>(ss.size());

    const int order =
        (npts >= 6) ? 3 :
        (npts >= 4) ? 2 : 1;

    std::vector<double> cx_s;

    std::vector<double> cy_s;

    if (
        !geom::polyFitW(ss, xs, ws, order, cx_s) ||
        !geom::polyFitW(ss, ys, ws, order, cy_s)
    )
    {
        return path;
    }


    // ------------------------------------------------------------
    // 5~6. 멱기저 -> Bernstein -> degree elevation
    //
    // BEV px -> base_link cm 환산이 여기 들어 있다. 중심선과 완전히
    // 같은 식이어야 좌/우/중심이 같은 좌표계로 나간다.
    // ------------------------------------------------------------

    const double h_px =
        static_cast<double>(bev_height);

    const double cx_bev =
        0.5 * static_cast<double>(bev_width);

    std::vector<kau::bezier::Point2> power(order + 1);

    double s_pow = 1.0;

    for (
        int k = 0;
        k <= order;
        ++k
    )
    {
        const double ax = cx_s[order - k];

        const double ay = cy_s[order - k];

        if (k == 0)
        {
            power[0].x =
                x_base + path_x_offset_cm_ +
                (h_px - ay) * sy;

            power[0].y =
                path_y_offset_cm_ +
                sx * (cx_bev - ax);
        }
        else
        {
            s_pow *= s_max;

            power[k].x = -sy * ay * s_pow;

            power[k].y = -sx * ax * s_pow;
        }
    }

    const kau::bezier::Ctrl fitted =
        kau::bezier::powerToBernstein(power);

    path.ctrl =
        kau::bezier::elevate(
            fitted,
            kau::bezier::DEGREE
        );


    // ------------------------------------------------------------
    // 6-b. 카메라 요(yaw) 회전 보정 — 중심선과 같은 피벗/부호
    // ------------------------------------------------------------

    if (std::abs(camera_yaw_deg) > 1e-9)
    {
        const double yaw_rad = camera_yaw_deg * M_PI / 180.0;

        const double cos_yaw = std::cos(yaw_rad);

        const double sin_yaw = std::sin(yaw_rad);

        for (kau::bezier::Point2 & cp : path.ctrl)
        {
            const double x_rel = cp.x - pan_pivot_offset_x_cm_;

            const double y_rel = cp.y - pan_pivot_offset_y_cm_;

            cp.x =
                pan_pivot_offset_x_cm_ +
                x_rel * cos_yaw - y_rel * sin_yaw;

            cp.y =
                pan_pivot_offset_y_cm_ +
                x_rel * sin_yaw + y_rel * cos_yaw;
        }
    }


    // ------------------------------------------------------------
    // 7. 퇴화 검사
    // ------------------------------------------------------------

    if (!kau::bezier::isRegular(path.ctrl))
    {
        return path;
    }


    // ------------------------------------------------------------
    // 8. 진단값
    //
    // confidence 근거량 30 은 중심선과 같은 상수를 쓴다 — 한 줄짜리
    // 추적이라 창 수가 중심선보다 적게 나오는 것은 사실이지만,
    // 하류가 세 경로의 confidence 를 같은 자로 비교해야 한다.
    // ------------------------------------------------------------

    path.source_windows = lane.found_count;

    path.length_cm =
        kau::bezier::segLength(path.ctrl);

    path.kappa_max =
        kau::bezier::kappaMaxExact(path.ctrl);

    path.valid_length_cm =
        kau::bezier::segLength(
            path.ctrl,
            0.0,
            cover_ratio
        );

    path.confidence =
        std::clamp(
            static_cast<double>(path.source_windows) / 30.0,
            0.0,
            1.0
        ) * cover_ratio;

    const kau::bezier::Nearest near =
        kau::bezier::nearestOnSeg(
            path.ctrl,
            kau::bezier::Point2{0.0, 0.0}
        );

    const kau::bezier::Point2 foot =
        kau::bezier::evalSeg(path.ctrl, near.u);

    const double th =
        kau::bezier::heading(path.ctrl, near.u);

    path.cte_cm =
        -std::sin(th) * (0.0 - foot.x) +
         std::cos(th) * (0.0 - foot.y);

    path.heading_err =
        std::remainder(-th, 2.0 * M_PI);

    path.built = true;


    // 좌/우 경로에는 중심선의 발행 게이트(pathGate)를 걸지 않는다.
    // 그 게이트는 "자차가 따라갈 경로" 기준(조향 한계 / 시야이탈)
    // 이라 도로 경계선에는 뜻이 맞지 않는다. 하류는 confidence 와
    // valid_length_cm 으로 신뢰도를 판단한다.
    path.valid = true;

    return path;
}


KauLaneDetectionNode::LanePath KauLaneDetectionNode::buildCenterlinePath(
    const LaneDetectionResult & left,
    const LaneDetectionResult & yellow,
    const LaneDetectionResult & right,
    int bev_width,
    int bev_height,
    double camera_yaw_deg,
    std::vector<cv::Point2d> & center_pts_px)
{
    LanePath path;

    center_pts_px.clear();


    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    if (!bevScale(sx, sy, x_base))
    {
        return path;
    }


    // ------------------------------------------------------------
    // 1. 세 차선 -> 중심선 후보
    // ------------------------------------------------------------

    struct Source
    {
        const LaneDetectionResult * lane;

        double offset;

        // 노란 중앙선은 관측 그대로다. 흰선 둘은 lane_width_px 만큼
        // 평행이동해 "중앙선이었을 자리" 를 추정한 값이다.
        bool observed_center;
    };


    const Source sources[3] =
    {
        { &left,    lane_width_px_, false },
        { &yellow,  0.0,            true  },
        { &right,  -lane_width_px_, false }
    };


    // center_source == "yellow" 면 추정 후보를 빼고 관측만 쓴다.
    // 하류가 /lane/left, /lane/right 로 corridor 를 직접 만들 때는
    // 여기서 섞는 것이 오히려 방해가 된다.
    const bool center_yellow_only =
        (center_source_ != "fused");


    struct Candidate
    {
        std::vector<cv::Point2d> pts;

        double weight;
    };


    std::vector<Candidate> cands;


    for (const Source & src : sources)
    {
        if (center_yellow_only && !src.observed_center)
        {
            continue;
        }


        if (
            !src.lane->valid ||
            src.lane->found_count <= 0 ||
            src.lane->track_px.size() < 2
        )
        {
            continue;
        }


        std::vector<cv::Point2d> c =
            geom::offsetTrack(
                src.lane->track_px,
                src.offset
            );


        if (c.size() < 2)
        {
            continue;
        }


        cands.push_back(
            Candidate{
                std::move(c),
                static_cast<double>(src.lane->found_count)
            }
        );
    }


    if (cands.empty())
    {
        return path;
    }


    // ------------------------------------------------------------
    // 2. 기준선(spine) 선정 + 누적 호길이
    //
    // 근거(잡은 스텝 수)가 가장 많은 후보를 기준으로 삼는다.
    // ------------------------------------------------------------

    std::size_t spine_i = 0;


    for (
        std::size_t i = 1;
        i < cands.size();
        ++i
    )
    {
        if (cands[i].weight > cands[spine_i].weight)
        {
            spine_i = i;
        }
    }


    const std::vector<cv::Point2d> & spine =
        cands[spine_i].pts;


    std::vector<double> cum(spine.size(), 0.0);


    for (
        std::size_t i = 1;
        i < spine.size();
        ++i
    )
    {
        cum[i] =
            cum[i - 1] +
            cv::norm(spine[i] - spine[i - 1]);
    }


    // ------------------------------------------------------------
    // 3. 이탈 후보 제거 + 기준선 투영으로 s 통일
    //
    // 노란선이 소실되면 좌/우 히스토그램이 같은 흰선을 무는
    // 일이 있다. 그러면 두 후보가 차로 폭만큼 어긋난 채로
    // 평균에 들어가 중심선이 통째로 밀린다.
    // ------------------------------------------------------------

    std::vector<double> ss;

    std::vector<double> xs;

    std::vector<double> ys;

    std::vector<double> ws;

    double total_weight = 0.0;

    const double outlier_px =
        center_outlier_ratio_ * lane_width_px_;


    for (
        std::size_t i = 0;
        i < cands.size();
        ++i
    )
    {
        std::vector<double> cs(cands[i].pts.size());

        std::vector<double> cd(cands[i].pts.size());


        for (
            std::size_t k = 0;
            k < cands[i].pts.size();
            ++k
        )
        {
            geom::projectOnPolyline(
                spine,
                cum,
                cands[i].pts[k],
                cs[k],
                cd[k]
            );
        }


        // 기준선 자신은 항상 남긴다.
        if (i != spine_i)
        {
            std::vector<double> sorted = cd;

            std::nth_element(
                sorted.begin(),
                sorted.begin() + sorted.size() / 2,
                sorted.end()
            );

            if (sorted[sorted.size() / 2] > outlier_px)
            {
                continue;
            }
        }


        total_weight += cands[i].weight;


        for (
            std::size_t k = 0;
            k < cands[i].pts.size();
            ++k
        )
        {
            ss.push_back(cs[k]);

            xs.push_back(cands[i].pts[k].x);

            ys.push_back(cands[i].pts[k].y);

            ws.push_back(cands[i].weight);
        }
    }


    if (
        total_weight <= 0.0 ||
        ss.size() < 2
    )
    {
        return path;
    }


    // 외삽으로 s 가 음수일 수 있다. 0 부터 시작하도록 평행이동.
    const double s_min =
        *std::min_element(ss.begin(), ss.end());


    for (double & v : ss)
    {
        v -= s_min;
    }


    const double s_max =
        *std::max_element(ss.begin(), ss.end());


    if (s_max < 1e-6)
    {
        return path;
    }


    path.source_windows =
        static_cast<int>(std::lround(total_weight));


    // ------------------------------------------------------------
    // 3-b. 실관측 구간
    //
    // 점선 공백을 관성 주행(track_max_miss)으로 건너뛰면 그 구간
    // 에는 근거가 없다. 근거가 처음 끊기는 지점까지가 "관측했다"
    // 고 말할 수 있는 범위이고, 그 뒤는 외삽이다.
    //
    // KauPath.valid_length 로 발행한다. 수신측이 신뢰 구간과
    // 외삽 구간을 구분할 수 있어야 하기 때문이다.
    // ------------------------------------------------------------

    double cover_ratio = 1.0;

    {
        std::vector<double> sorted_s = ss;

        std::sort(
            sorted_s.begin(),
            sorted_s.end()
        );


        // 스텝 2개분 이상 비면 근거가 끊긴 것으로 본다
        const double gap_limit =
            2.0 * std::max(4.0, track_step_px_);

        double s_cover = s_max;


        for (
            std::size_t k = 1;
            k < sorted_s.size();
            ++k
        )
        {
            if (sorted_s[k] - sorted_s[k - 1] > gap_limit)
            {
                s_cover = sorted_s[k - 1];

                break;
            }
        }


        cover_ratio =
            std::clamp(s_cover / s_max, 0.0, 1.0);
    }


    // ------------------------------------------------------------
    // 4. 내부 적합 x(s), y(s)
    // ------------------------------------------------------------

    const int npts =
        static_cast<int>(ss.size());

    const int order =
        (npts >= 6) ? 3 :
        (npts >= 4) ? 2 : 1;


    std::vector<double> cx_s;

    std::vector<double> cy_s;


    if (
        !geom::polyFitW(ss, xs, ws, order, cx_s) ||
        !geom::polyFitW(ss, ys, ws, order, cy_s)
    )
    {
        return path;
    }


    // 시각화용 중심선 점열 (연산에는 쓰지 않는다)
    for (
        int i = 0;
        i <= NUM_WINDOWS;
        ++i
    )
    {
        const double s =
            s_max * i / NUM_WINDOWS;

        center_pts_px.push_back(
            cv::Point2d(
                geom::polyEval(cx_s, s),
                geom::polyEval(cy_s, s)
            )
        );
    }


    // ------------------------------------------------------------
    // 5. 멱기저 계수 (매개변수 t in [0,1],  s = s_max * t)
    //
    // BEV px -> 차량 좌표 [cm] 는 아핀 변환이다.
    //
    //   X(전방) = x_base + x_off + (H - y_px) * sy
    //   Y(좌)   = y_off  + (cx_bev - x_px) * sx
    //
    // x_px(s), y_px(s) 가 s 의 다항식이므로 X(t), Y(t) 도
    // t 의 같은 차수 다항식이다. 근사가 개입하지 않는다.
    //
    // polyFitW 의 계수 배열은 최고차부터이므로
    // s^k 계수는 coeffs[order - k] 다.
    // ------------------------------------------------------------

    const double h_px =
        static_cast<double>(bev_height);

    const double cx_bev =
        0.5 * static_cast<double>(bev_width);


    std::vector<kau::bezier::Point2> power(order + 1);


    double s_pow = 1.0;


    for (
        int k = 0;
        k <= order;
        ++k
    )
    {
        const double ax = cx_s[order - k];

        const double ay = cy_s[order - k];


        if (k == 0)
        {
            power[0].x =
                x_base + path_x_offset_cm_ +
                (h_px - ay) * sy;

            power[0].y =
                path_y_offset_cm_ +
                sx * (cx_bev - ax);
        }
        else
        {
            s_pow *= s_max;

            power[k].x = -sy * ay * s_pow;

            power[k].y = -sx * ax * s_pow;
        }
    }


    // ------------------------------------------------------------
    // 6. 멱기저 -> Bernstein -> degree elevation (order -> 5)
    //
    // 둘 다 정확한 기저 변환이라 무손실이다.
    // ------------------------------------------------------------

    const kau::bezier::Ctrl fitted =
        kau::bezier::powerToBernstein(power);

    path.ctrl =
        kau::bezier::elevate(
            fitted,
            kau::bezier::DEGREE
        );


    // ------------------------------------------------------------
    // 6-b. 카메라 요(yaw) 회전 보정 (pan 중에도 경로를 짓는 이유)
    //
    // 여기까지의 좌표는 "지금 카메라가 보는 방향" 기준이다
    // (선언부 헤더 §8 도입부 주석 참고 — bevScale() 의 row->전방
    // 거리 / column->횡거리 유도가 intrinsic 과 소실점 행만 쓰므로
    // 순수 요 회전에 불변이다. pan != 0 이어도 이 자체는 그대로
    // 유효한 카메라 기준 좌표다). camera_yaw_deg 만큼 pan 피벗을
    // 중심으로 회전하면 base_link 기준이 된다.
    //
    // Bezier 는 아핀변환(회전+평행이동)에 닫혀 있으므로 제어점만
    // 이렇게 옮기면 곡선 전체가 옮겨진다 — 다시 적합할 필요가 없다.
    //
    // camera_yaw_deg == 0(Idle) 이면 cos=1, sin=0 이라 항등변환다 —
    // 예전 동작과 완전히 같다.
    // ------------------------------------------------------------

    if (std::abs(camera_yaw_deg) > 1e-9)
    {
        const double yaw_rad = camera_yaw_deg * M_PI / 180.0;

        const double cos_yaw = std::cos(yaw_rad);

        const double sin_yaw = std::sin(yaw_rad);

        for (kau::bezier::Point2 & cp : path.ctrl)
        {
            const double x_rel = cp.x - pan_pivot_offset_x_cm_;

            const double y_rel = cp.y - pan_pivot_offset_y_cm_;

            cp.x =
                pan_pivot_offset_x_cm_ +
                x_rel * cos_yaw - y_rel * sin_yaw;

            cp.y =
                pan_pivot_offset_y_cm_ +
                x_rel * sin_yaw + y_rel * cos_yaw;
        }
    }


    // ------------------------------------------------------------
    // 7. 퇴화 검사 (문서 8.7) — 발행 직전 필수
    // ------------------------------------------------------------

    if (!kau::bezier::isRegular(path.ctrl))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Degenerate path (cusp). not published."
        );

        return path;
    }


    // ------------------------------------------------------------
    // 8. 진단값
    //
    // 최근접점은 9차 다항식 실근 전체 + 단부 (문서 8.1).
    // 차량 원점 (0,0) 에 대해 풀면 그대로 횡오차가 된다.
    // ------------------------------------------------------------

    path.length_cm =
        kau::bezier::segLength(path.ctrl);

    path.kappa_max =
        kau::bezier::kappaMaxExact(path.ctrl);


    // 근거가 끊기기 전까지의 호길이. 그 뒤는 외삽이다.
    path.valid_length_cm =
        kau::bezier::segLength(
            path.ctrl,
            0.0,
            cover_ratio
        );


    // 근거량 x 관측 비율.
    //
    // 근거량은 세 차선이 내놓은 점의 총 개수다. 실측 중앙값이
    // 29개이므로 30을 만점으로 둔다 (직선 구간에서 포화).
    // 관측 비율은 위 3-b 의 cover_ratio.
    path.confidence =
        std::clamp(
            static_cast<double>(path.source_windows) / 30.0,
            0.0,
            1.0
        ) * cover_ratio;


    const kau::bezier::Nearest near =
        kau::bezier::nearestOnSeg(
            path.ctrl,
            kau::bezier::Point2{0.0, 0.0}
        );

    const kau::bezier::Point2 foot =
        kau::bezier::evalSeg(path.ctrl, near.u);

    const double th =
        kau::bezier::heading(path.ctrl, near.u);

    // 좌측 +
    path.cte_cm =
        -std::sin(th) * (0.0 - foot.x) +
         std::cos(th) * (0.0 - foot.y);

    // 차량 yaw 는 자기 frame 에서 0
    path.heading_err =
        std::remainder(-th, 2.0 * M_PI);


    path.built = true;


    // ------------------------------------------------------------
    // 9. 발행 게이트
    //
    // 기각해도 ctrl 은 그대로 둔다. debug 화면에 붉게 그려서
    // "무엇이 막혔는지" 가 보여야 원인을 좁힐 수 있다.
    // ------------------------------------------------------------

    path.reject = pathGate(path, sx, camera_yaw_deg);

    if (path_gate_enable_ && path.reject != 0)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "경로 기각(%s): 창 %d개, 반경 %.0fcm",
            gateLabel(path.reject).ko,
            path.source_windows,
            (path.kappa_max > 1e-9) ? 1.0 / path.kappa_max : 0.0
        );

        return path;
    }


    path.valid = true;


    return path;
}
