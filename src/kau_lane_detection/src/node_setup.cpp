// ====================================================================
// node_setup.cpp
//
// 생성자. 파라미터 선언, 퍼블리셔/서브스크립션 배선, 기동 로그.
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
// Constructor
// ================================================================

KauLaneDetectionNode::KauLaneDetectionNode()
    : Node("kau_lane_detection_node"),
      camera_calibrated_(false)
{
    // ============================================================
    // Subscriber
    // ============================================================

    image_subscriber_ =
        this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                guardCallback(
                    "imageCallback",
                    [&] { imageCallback(msg); }
                );
            }
        );

    camera_info_subscriber_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",
            10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                guardCallback(
                    "cameraInfoCallback",
                    [&] { cameraInfoCallback(msg); }
                );
            }
        );

    // 전역 경로. kau_global_path 가 latched(RELIABLE + TRANSIENT_LOCAL)
    // 로 한 번 낸다. 이 노드가 늦게 떠도 같은 QoS 라야 마지막 값을 받는다.
    global_path_subscriber_ =
        this->create_subscription<kau_msgs::msg::KauPath>(
            "/path/global",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const kau_msgs::msg::KauPath::SharedPtr msg)
            {
                guardCallback(
                    "globalPathCallback",
                    [&] { globalPathCallback(msg); }
                );
            }
        );

    // 장애물 가림 판정용. kau_object_detection 발행측과 QoS 를 맞춘다
    // (BEST_EFFORT, depth 1, volatile).
    obstacles_subscriber_ =
        this->create_subscription<kau_msgs::msg::ObstacleCircleArray>(
            "/perception/obstacles",
            rclcpp::QoS(1).best_effort(),
            [this](const kau_msgs::msg::ObstacleCircleArray::SharedPtr msg)
            {
                guardCallback(
                    "obstaclesCallback",
                    [&] { obstaclesCallback(msg); }
                );
            }
        );

    // camera_pan_joint 실측 각도 (Pan 탐색 정착 판정용, 50Hz)
    joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                guardCallback(
                    "jointStateCallback",
                    [&] { jointStateCallback(msg); }
                );
            }
        );


    // map <- base_link. 경로를 map 프레임으로 내보내고 pan 트리거의
    // 곡률/장애물 판정에 쓴다 (측위 의존, CLAUDE.md §1 개정 근거 참고).
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);


    // ============================================================
    // Publisher
    // ============================================================
    path_max_step_cm_per_s_ =
    this->declare_parameter<double>(
        "path_max_step_cm_per_s",
        80.0
    );
    undistorted_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/undistorted_image",
            10
        );

    bev_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/bev_image",
            10
        );

    debug_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/debug_image",
            10
        );

    // 검출 상태 진단 (실패 프레임 수집용)
    status_publisher_ =
        this->create_publisher<std_msgs::msg::String>(
            "/kau_lane_detection/status",
            10
        );


    roi_overlay_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/bev_roi",
            10
        );


    // Pan 기반 Lane Lost Recovery 명령 채널.
    // physicar_driver_node 의 apply_pan() 이 절대각[rad]으로 받는다.
    camera_pan_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>(
            "/camera/pan",
            10
        );


    // 기동 tilt — BEV 세 행이 전제하는 카메라 pitch 를 실제로 세운다.
    // 제어량이 아니라 고정 자세다 (CLAUDE.md §12).

    camera_tilt_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>(
            "/camera/tilt",
            10
        );

    camera_tilt_enable_ =
        this->declare_parameter<bool>(
            "camera_tilt_enable",
            true
        );

    // 기본 0 — 값의 주인은 yaml 이다.
    //
    // 예전 기본값은 5.0 이었는데 yaml 은 10.0 이라, 둘이 갈라진 채로
    // "카메라는 10도로 서 있고 BEV 세 행은 5도를 전제" 하는 상태가
    // 오래 굴러갔다. 0 이면 yaml 을 안 주는 순간 카메라가 중립에
    // 서므로 어긋남이 즉시 눈에 보인다 — 조용히 틀린 각도로 도는
    // 것보다 낫다.
    camera_tilt_deg_ =
        this->declare_parameter<double>(
            "camera_tilt_deg",
            0.0
        );

    // 토픽 배선 부호. 기본은 시뮬(+ = 아래)이라 1.0 이다.
    // 실차는 launch 의 platform:=real 이 -1.0 으로 덮는다.
    // 외부 /camera/tilt 를 만나면 양보할지. 기본은 무시하고 재설정.
    camera_tilt_yield_enable_ =
        this->declare_parameter<bool>(
            "camera_tilt_yield_enable",
            false
        );

    camera_tilt_sign_ =
        this->declare_parameter<double>(
            "camera_tilt_sign",
            1.0
        );

    camera_tilt_repeat_s_ =
        this->declare_parameter<double>(
            "camera_tilt_repeat_s",
            5.0
        );

    // 지금 yaml 의 BEV 세 행은 이 각도를 전제하고 유도된 값이다.
    camera_tilt_bev_ref_deg_ = camera_tilt_deg_;

    // 남이 /camera/tilt 를 잡으면 손을 뗀다 (아래 tiltEchoCallback).
    camera_tilt_echo_subscriber_ =
        this->create_subscription<std_msgs::msg::Float64>(
            "/camera/tilt",
            10,
            std::bind(
                &KauLaneDetectionNode::tiltEchoCallback,
                this,
                std::placeholders::_1
            )
        );

    if (camera_tilt_enable_)
    {
        // wall timer 를 쓰는 것은 의도적이다 — node clock 은
        // use_sim_time 과 센서 스탬프가 어긋나는 이 스택에서
        // 신뢰할 수 없다 (§12 마지막 항목).
        camera_tilt_ticks_left_ =
            std::max(
                1,
                static_cast<int>(std::lround(camera_tilt_repeat_s_))
            );

        camera_tilt_timer_ =
            this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(
                    &KauLaneDetectionNode::publishCameraTilt,
                    this
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "기동 tilt: 아래로 %.2f deg. 토픽에는 %+.2f deg 를 %d 회 "
            "발행한다 (camera_tilt_sign %+.1f). bev_vanishing_y 가 "
            "이 하향각을 전제한다.",
            camera_tilt_deg_,
            camera_tilt_sign_ * camera_tilt_deg_,
            camera_tilt_ticks_left_,
            camera_tilt_sign_
        );
    }


    // ============================================================
    // BEV 파라미터
    //
    //   src : 원본(undistort)에서 잘라낼 사다리꼴
    //   dst : BEV 에서 그 사다리꼴이 놓일 사각형
    //
    // 윗변 폭은 주지 않고 소실점에서 유도한다:
    //   top_width = bottom_width * (top_y - vanishing_y)
    //                            / (bottom_y - vanishing_y)
    // ============================================================

    bev_src_center_x_ =
        this->declare_parameter<double>(
            "bev_src_center_x",
            240.0
        );

    // 윗변 y. 내릴수록 멀리 본다 (CLAUDE.md §13).
    bev_src_top_y_ =
        this->declare_parameter<double>(
            "bev_src_top_y",
            210.0
        );

    bev_src_bottom_y_ =
        this->declare_parameter<double>(
            "bev_src_bottom_y",
            270.0
        );


    // ------------------------------------------------------------
    // 소실점 y
    //
    // BEV에서 위쪽이 아래쪽보다 좁으면 -> 값을 올린다
    // BEV에서 위쪽이 아래쪽보다 넓으면 -> 값을 내린다
    // ------------------------------------------------------------

    bev_vanishing_y_ =
        this->declare_parameter<double>(
            "bev_vanishing_y",
            179.0
        );

    bev_src_bottom_width_ =
        this->declare_parameter<double>(
            "bev_src_bottom_width",
            480.0
        );


    // ------------------------------------------------------------
    // dst 좌우 여백 비율
    //
    // 값이 클수록 차선이 가운데로 모인다.
    //
    // 0.0  -> 차선이 화면 양 끝까지 꽉 참
    // 0.18 -> 좌/우 흰선이 약 101px, 379px
    // 0.25 -> 좌/우 흰선이 약 129px, 351px
    // ------------------------------------------------------------

    bev_dst_margin_ratio_ =
        this->declare_parameter<double>(
            "bev_dst_margin_ratio",
            0.25
        );

    bev_out_width_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "bev_out_width",
                480
            )
        );

    bev_out_height_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "bev_out_height",
                360
            )
        );


    // 슬라이딩 윈도우 횡 반폭 [BEV px] (CLAUDE.md §13).

    window_margin_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "window_margin",
                40
            )
        );


    // HLS 색 임계 (H, L, S). 근거는 CLAUDE.md §4.

    yellow_hls_lo_ =
        this->declare_parameter<std::vector<int64_t>>(
            "yellow_hls_lo",
            {0, 80, 105}
        );

    yellow_hls_hi_ =
        this->declare_parameter<std::vector<int64_t>>(
            "yellow_hls_hi",
            {20, 255, 255}
        );

    white_hls_lo_ =
        this->declare_parameter<std::vector<int64_t>>(
            "white_hls_lo",
            {0, 205, 0}
        );

    white_hls_hi_ =
        this->declare_parameter<std::vector<int64_t>>(
            "white_hls_hi",
            {179, 255, 255}
        );


    // ------------------------------------------------------------
    // Window 판정 최소 픽셀 수
    //
    // 창 크기가 (2*margin) x (h/9) = 50 x 40 = 2000px 이라
    // 예전 값 5 는 노이즈 몇 점에도 창 중심이 끌려갔다.
    // 곡선에서 창이 엉뚱한 곳으로 끌려가면 그대로 끊긴다.
    //
    // 곡선에서 자꾸 놓치면 내리고, 노이즈에 끌리면 올린다.
    // ------------------------------------------------------------

    min_pixels_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "min_pixels",
                20
            )
        );


    // ------------------------------------------------------------
    // 방향성 추적 — 창을 진행 방향으로 회전시키며 전진한다.
    // 이 넷이 "90도 코너에서 차선이 이어지는가" 를 정한다 (CLAUDE.md §5).
    // ------------------------------------------------------------

    // 한 스텝 전진 거리 [BEV px]. 코너 추종을 지배하는 값.
    track_step_px_ =
        this->declare_parameter<double>(
            "track_step_px",
            25.0
        );


    // 한 스텝에서 허용하는 최대 방향 변화.
    //
    // 너무 키우면 옆 차선이나 노이즈를 물었을 때 창이 되꺾여
    // 왔던 길을 되돌아간다.
    track_max_turn_deg_ =
        this->declare_parameter<double>(
            "track_max_turn_deg",
            30.0
        );


    // 최대 스텝 수.
    //
    // step 25px 이면 직선으로 BEV 세로(360px)를 지나는 데
    // 14스텝이다. 코너는 같은 거리를 돌아가느라 더 들고,
    // 여기서 잘리면 경로가 그만큼 짧아진다.
    track_max_steps_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "track_max_steps",
                48
            )
        );


    // 연속으로 비어도 관성 주행할 스텝 수.
    //
    // 노란 중앙선이 점선이라 공백을 건너뛰어야 한다.
    // 너무 키우면 차선이 끝난 뒤에도 허공을 계속 걷는다.
    track_max_miss_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "track_max_miss",
                4
            )
        );


    // 흰선 좌/우 판정 [lane_width_px 비율]. 추적이 끝난 흰선이 노란선
    // 기준으로 제 쪽에 있는지 본다 (CLAUDE.md §5).
    lane_side_min_ratio_ =
        this->declare_parameter<double>(
            "lane_side_min_ratio",
            0.40
        );


    // 중심선 후보 이탈 판정 [lane_width_px 비율]. 근거가 가장 많은
    // 후보에서 이 비율 이상 떨어진 후보는 버린다 (CLAUDE.md §5).
    center_outlier_ratio_ =
        this->declare_parameter<double>(
            "center_outlier_ratio",
            0.30
        );


    // 흰선 회랑 [lane_width 비율]. 이탈한 지점에서 자른다 (CLAUDE.md §5).

    lane_corridor_lo_ =
        this->declare_parameter<double>(
            "lane_corridor_lo",
            0.60
        );

    lane_corridor_hi_ =
        this->declare_parameter<double>(
            "lane_corridor_hi",
            1.40
        );


    // 차로 폭 [BEV px] — 파라미터가 아니라 lane_width_cm / sx 유도값.
    // 아래 초기값은 CameraInfo 도착 전까지만 쓰이는 자리표시다
    // (imageCallback 이 CameraInfo 없이는 조기 반환한다).

    lane_width_px_ = 100.0;


    // 히스토그램 밴드 높이 [BEV 하단 비율]. 비면 넓혀 재시도한다
    // (findHistogramPeak). 실측표는 CLAUDE.md §13.

    histogram_band_ratio_ =
        this->declare_parameter<double>(
            "histogram_band_ratio",
            0.10
        );


    // src 사다리꼴 오버레이를 debug 이미지에 그릴지
    publish_roi_overlay_ =
        this->declare_parameter<bool>(
            "publish_roi_overlay",
            true
        );


    // 발행 frame. X 전방 / Y 좌 / 단위 cm.
    //
    // "map" 이면 publishLanePath() 가 발행 직전에 map <- base_link TF 로
    // 제어점만 옮긴다 — 내부 계산은 전부 base_link 기준 그대로다.
    // TF 조회 실패 시 그 프레임은 발행을 건너뛴다 (CLAUDE.md §1).
    path_frame_id_ =
        this->declare_parameter<std::string>(
            "path_frame_id",
            "base_link"
        );


    publish_path_ =
        this->declare_parameter<bool>(
            "publish_path",
            true
        );


    // ------------------------------------------------------------
    // 실 차로 폭 [cm]  <-- 차선 정보의 본체
    //
    // 노란 중앙선 중심 <-> 흰 차선 중심 실제 거리.
    //
    // 축척에는 쓰이지 않는다 (축척은 camera_height_cm 이 정한다).
    // 이 값은 lane_width_px 로 환산되어 흰선 탐색 밴드, 좌/우
    // 판정, 회랑 절단, 한쪽 소실 복원, 그리고 identifyLine 의
    // 기준이 된다.
    //
    // 31.25 는 맵 mesh 실측값이다. 트랙 .dae 의 삼각형을
    // x = 6.0 m 평면으로 잘라 단면을 냈다 (단위 m):
    //
    //   road   y 0.8805 .. 1.5805    아스팔트 폭 0.700
    //   ol     y 0.8805 .. 0.9555    흰 실선   폭 0.075
    //   cl     y 1.2180 .. 1.2430    황색 점선 폭 0.025
    //   il     y 1.5055 .. 1.5805    흰 실선   폭 0.075
    //
    //   중앙선 중심 <-> 흰선 중심 0.3125 m
    //   흰선 중심   <-> 흰선 중심 0.6250 m
    //
    // 영상 검증: 직선 구간 한 프레임의 BEV 에서 흰선 사이
    // 간격이 195.0 px 로 일정했고, sx = 0.3220 cm/px 이므로
    // 62.79 cm. mesh 실측 62.50 과 0.5% 차이.
    //
    // (이전에 35.0 이 들어 있었다. 곡선 구간이 섞인 주행
    //  로그에서 흰선-노란선 간격 중앙값 108.3 px 를 참값으로
    //  본 결과인데, 그 실측은 노란선 폴리라인의 현(弦)에
    //  투영해 재는 방식이라 곡선 바깥쪽에서 부풀려진다.
    //  직선 프레임에서 다시 재면 97.5 px 다.)
    // ------------------------------------------------------------

    lane_width_cm_ =
        this->declare_parameter<double>(
            "lane_width_cm",
            kau_road::LANE_WIDTH_CM
        );


    // ------------------------------------------------------------
    // 카메라 높이 [cm] - 축척의 유일한 기준
    //
    // BEV 의 가로/세로 축척, 경로 X 원점, lane_width_px 가
    // 전부 이 값 하나에서 나온다. 반드시 실측값을 넣을 것.
    //
    // 기본값 14.65 는 시뮬레이터 모델에서 포즈 체인을 합산한 값:
    //
    //   model.sdf (models/physicar/model.sdf)
    //     camera_pan_joint   rel base_footprint : z 0.1195
    //     camera_tilt_joint  rel camera_pan_link: z 0.0130
    //     camera_sensor      rel camera_tilt_link: z 0.0140
    //                                        합계  0.1465 m
    //
    //   base_footprint 는 지면. 바퀴축 z 0.0375 = 바퀴 반지름 0.0375
    //
    // 검증: 직선 구간 한 프레임을 undistort 해 지면으로
    // 역투영하고, 맵 mesh 실측 치수와 맞춰 h 를 역산하면
    //
    //   도로 바깥 가장자리 간격  70.0 cm -> h 14.552
    //   흰선 안쪽 가장자리 간격  55.0 cm -> h 14.670
    //   중앙선 <-> 왼쪽 흰선 안쪽 27.5 cm -> h 14.708
    //   중앙선 <-> 오른쪽 흰선 안쪽 27.5 cm -> h 14.633
    //
    // 네 경로가 모두 14.6 근처로 모인다.
    //
    // 실차에서는 지면부터 렌즈 중심까지 자로 재서 넣는다.
    // ------------------------------------------------------------

    camera_height_cm_ =
        this->declare_parameter<double>(
            "camera_height_cm",
            14.65
        );


    // ------------------------------------------------------------
    // 카메라 접지점 -> path frame 원점 보정 [cm]
    // ------------------------------------------------------------

    path_x_offset_cm_ =
        this->declare_parameter<double>(
            "path_x_offset_cm",
            0.0
        );

    path_y_offset_cm_ =
        this->declare_parameter<double>(
            "path_y_offset_cm",
            0.0
        );


    // ------------------------------------------------------------
    // 발행한 Bezier 를 debug_image 에 되돌려 그릴지
    // ------------------------------------------------------------

    draw_path_overlay_ =
        this->declare_parameter<bool>(
            "draw_path_overlay",
            true
        );


    // ============================================================
    // 도로 모형. 단면의 형태는 road_map.hpp 가, 크기는 lane_width_cm
    // 이 정한다. 차로 번호 개념은 없다 (CLAUDE.md §2, §0-3).
    // ============================================================


    // 선언된 단면을 기동 로그에 남긴다. 맵을 바꿔 놓고
    // road_map.hpp 를 안 고친 경우가 여기서 드러난다.
    {
        std::string cross;

        for (
            std::size_t i = 0;
            i < kau_road::LINE_COUNT;
            ++i
        )
        {
            char one[64];

            std::snprintf(
                one,
                sizeof(one),
                "%s%+.2fcm %s",
                (i ? " | " : ""),
                lineOffsetCm(i),
                kau_road::LINES[i].name
            );

            cross += one;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "차선 정보: 닫힌 루프 %d차선 (단면은 road_map.hpp), "
            "차로 폭 %.2fcm, 도로 폭 %.2fcm (yaml lane_width_cm 기준), "
            "차로 조건 없이 중앙선 추종 | 단면 %s",
            kau_road::LANE_COUNT,
            lane_width_cm_,
            2.0 * roadHalfWidthCm(),
            cross.c_str()
        );
    }


    // 근거(노란선 또는 흰선 두 개)가 끊겨도 직전 중앙선 대비
    // 위치 판정을 유지할 프레임 수. 점선 공백과 순간 가림 대응.
    center_source_ =
        this->declare_parameter<std::string>(
            "center_source",
            "yellow"
        );

    center_hold_frames_ =
        this->declare_parameter<int>(
            "center_hold_frames",
            10
        );


    // ============================================================
    // 발행 게이트 — 근거가 모자란 경로를 막는다.
    // 창 3개면 2차 적합의 자유도가 0 이라 잔차로는 안 걸러진다
    // (CLAUDE.md §13).
    // ============================================================

    path_gate_enable_ =
        this->declare_parameter<bool>(
            "path_gate_enable",
            true
        );


    // 중심선이 딛고 선 실측 창 수 하한.
    //
    // 2차 적합에 계수가 3개다. 3창이면 자유도 0.
    // 6창이면 2배 과결정이라 이상치가 드러난다.
    //
    // 실측: 정상 구간 25~27창, 실패 프레임 중앙값 5창.
    min_path_windows_ =
        this->declare_parameter<int>(
            "min_path_windows",
            6
        );


    // 차량 제원. 최소 회전반경 = wheelbase / tan(max_steer)
    wheelbase_cm_ =
        this->declare_parameter<double>(
            "wheelbase_cm",
            18.0
        );

    max_steer_deg_ =
        this->declare_parameter<double>(
            "max_steer_deg",
            20.0
        );


    // 경로가 벗어나면 안 되는 횡방향 한계 [cm].
    // 0 이면 BEV 영상 폭 절반으로 자동 설정한다.
    //
    // 근거: 본 적 없는 영역을 단언하지 않는다.
    path_gate_steer_enable_ =
        this->declare_parameter<bool>(
            "path_gate_steer_enable",
            false
        );

    max_lateral_cm_ =
        this->declare_parameter<double>(
            "max_lateral_cm",
            0.0
        );


    // ============================================================
    // Pan 트리거 판정 — 곡률 룩어헤드 + 장애물 (선언부 헤더 주석 참고)
    // ============================================================

    obstacle_pan_block_enable_ =
        this->declare_parameter<bool>(
            "obstacle_pan_block_enable",
            true
        );

    // 자차 위치에서 /path/global 전방 이만큼 [m] 을 살펴 최대 곡률을
    // 잰다. 최소회전반경(~50cm)과 예전 pan 트리거가 반응하던 거리
    // (BEV 룩어헤드 ~85cm) 사이 여유값. 실측 후 조정.
    curvature_lookahead_m_ =
        this->declare_parameter<double>(
            "curvature_lookahead_m",
            1.5
        );

    // 위 구간의 최대 곡률 [1/cm] 이 이 이상이면 "원래 급커브" 로 보고
    // 장애물 판정 없이 pan 탐색을 진행한다 (곡률 우선). 근거: 실측
    // 전 초안값 (R ~ 100cm). 튜닝 필요.
    curve_ahead_kappa_thresh_ =
        this->declare_parameter<double>(
            "curve_ahead_kappa_thresh",
            0.005
        );

    // 가려지지 않은 쪽(노란 중앙선)의 꺾임 [deg] 이 이 이상이면
    // 역시 급커브로 본다. 측위/전역경로 없이도 동작하는 근거다
    // (선언부 헤더 주석 참고). 근거: 실측 전 초안값. 튜닝 필요.
    curve_ahead_bend_deg_thresh_ =
        this->declare_parameter<double>(
            "curve_ahead_bend_deg_thresh",
            5.0
        );

    // 소실된 쪽 전방 이 거리 [m] 안에 장애물 원이 있으면 가림으로
    // 본다 (곡률이 급커브를 가리키지 않을 때만). 근거: 실측 전
    // 초안값. 튜닝 필요.
    obstacle_ahead_max_m_ =
        this->declare_parameter<double>(
            "obstacle_ahead_max_m",
            1.0
        );

    // 장애물 원이 차량 종축에서 이 횡거리 [m] 이내여야 우리 차로
    // 안으로 본다. 트랙 경계벽은 차로 폭 밖이라 이 조건을 구조적
    // 으로 통과하지 못한다 (근거: CLAUDE.md).
    obstacle_lane_lateral_m_ =
        this->declare_parameter<double>(
            "obstacle_lane_lateral_m",
            0.5
        );

    // /perception/obstacles 가 이보다 오래되면 판단하지 않는다 [s].
    obstacle_scan_timeout_s_ =
        this->declare_parameter<double>(
            "obstacle_scan_timeout_s",
            0.5
        );


    // ============================================================
    // 시간축 강건화 (EMA)
    //
    // 게이트 하나로는 오탐과 미탐을 동시에 못 잡는다.
    // 선언부(헤더) 주석에 근거를 적어 두었다.
    // ============================================================

    path_ema_enable_ =
        this->declare_parameter<bool>(
            "path_ema_enable",
            true
        );

    // confidence = 1 인 프레임의 혼합 비율.
    // 0.4 면 시상수가 약 2 프레임(14 Hz 에서 0.15 s)이다.
    path_ema_alpha_ =
        this->declare_parameter<double>(
            "path_ema_alpha",
            0.4
        );

    // 제어점 평균거리 기준. 차로 폭(31.25)의 절반쯤을 상한으로 둔다.
    // **공칭 14 Hz 기준값**이고, 느린 프레임에서는 dt 에 비례해
    // 넓혀 쓴다 (robustifyPath 주석 참고).
    path_ema_gate_cm_ =
        this->declare_parameter<double>(
            "path_ema_gate_cm",
            15.0
        );

    path_ema_gate_dt_scale_max_ =
        this->declare_parameter<double>(
            "path_ema_gate_dt_scale_max",
            6.0
        );

    path_ema_relock_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "path_ema_relock_frames",
                3
            )
        );

    // 14 Hz 이므로 14 프레임 = 1 초.
    path_ema_reset_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "path_ema_reset_frames",
                14
            )
        );


    // ============================================================
    // Pan 소실 탐색 (CLAUDE.md §8). 기본 경로는 §8-b 조준이고
    // 이쪽은 조준이 불가할 때의 fallback 이다.
    // ============================================================

    pan_search_enable_ =
        this->declare_parameter<bool>(
            "pan_search_enable",
            false
        );

    // 탐색 첫 목표각. 도달은 pan_rate_deg_s_ 램프로 한다
    // (예전에는 한 프레임에 점프했다). 그 뒤 pan_step_deg_ 씩 전진.
    pan_start_deg_ =
        this->declare_parameter<double>(
            "pan_start_deg",
            15.0
        );

    // 목표각으로 다가가는 각속도 상한 [deg/s]. 근거는 선언부 주석.
    pan_rate_deg_s_ =
        this->declare_parameter<double>(
            "pan_rate_deg_s",
            30.0
        );

    // 시작 각도에서 못 찾았을 때의 한 스텝 회전량.
    pan_step_deg_ =
        this->declare_parameter<double>(
            "pan_step_deg",
            3.0
        );

    // 서보 하드리밋 ±30도 (physicar_driver_node MAX_PAN,
    // URDF/SDF limit ±0.5236 rad). 그 안에서 전부 쓴다.
    pan_max_deg_ =
        this->declare_parameter<double>(
            "pan_max_deg",
            30.0
        );

    pan_settle_tol_deg_ =
        this->declare_parameter<double>(
            "pan_settle_tol_deg",
            1.0
        );

    pan_trigger_miss_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_trigger_miss_frames",
                3
            )
        );

    // 재검출 인정 기준 (선언부 주석 참고).
    pan_found_min_windows_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_found_min_windows",
                6
            )
        );

    pan_found_confirm_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_found_confirm_frames",
                2
            )
        );

    // 실측 전 추정값. 첫 테스트에서 카메라가 명령과 반대로
    // 돌면 -1.0 으로 바꾼다.
    pan_sign_ =
        this->declare_parameter<double>(
            "pan_sign",
            1.0
        );


    // 재검출 직후 추가로 더 돌릴 각도 [deg].
    // 기본 0 = 찾은 그 각도에서 그대로 정지.
    pan_overshoot_deg_ =
        this->declare_parameter<double>(
            "pan_overshoot_deg",
            0.0
        );


    // 재검출 후 복귀 조건 (Holding). updatePanSearch 주석 참고.
    // 화면 가장자리 여유 [deg]. 겨우 걸친 선은 "보인다" 로
    // 치지 않는다 (차가 조금만 움직여도 다시 빠진다).
    pan_return_fov_margin_deg_ =
        this->declare_parameter<double>(
            "pan_return_fov_margin_deg",
            3.0
        );

    pan_return_confirm_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_return_confirm_frames",
                3
            )
        );

    // 직선 판정에 필요한 최소 추적점 수.
    //
    // 짧은 점열은 어떤 코너에서도 직선으로 보인다. 근거가
    // 모자라면 "직선이 아니다" 가 아니라 "판정 불가" 로 다뤄
    // 연속 카운트를 끊는다 (Holding 유지).
    pan_return_min_points_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_return_min_points",
                10
            )
        );

    // 0 이하 = 무한 대기 (직선을 만날 때까지 복귀하지 않는다).
    pan_hold_timeout_s_ =
        this->declare_parameter<double>(
            "pan_hold_timeout_s",
            0.0
        );


    // ------------------------------------------------------------
    // Pan 조준 (feedforward). 선언부(헤더) panAimGoalDeg 주석 참고.
    //
    // 여기서는 declare 만 한다. 실제 값은 updatePanSearch 가 매
    // 프레임 get_parameter 로 다시 읽으므로 ros2 param set 이 즉시
    // 먹는다 — 되돌릴 때 재빌드가 필요 없어야 하기 때문이다.
    // ------------------------------------------------------------
    pan_aim_enable_ =
        this->declare_parameter<bool>(
            "pan_aim_enable",
            true
        );

    pan_aim_lookahead_m_ =
        this->declare_parameter<double>(
            "pan_aim_lookahead_m",
            1.0
        );

    pan_aim_gain_ =
        this->declare_parameter<double>(
            "pan_aim_gain",
            1.0
        );

    // 조준 근거. "lane" (기본) / "global" (예전 동작).
    pan_aim_source_ =
        this->declare_parameter<std::string>(
            "pan_aim_source",
            "lane"
        );

    pan_aim_min_evidence_cm_ =
        this->declare_parameter<double>(
            "pan_aim_min_evidence_cm",
            25.0
        );

    pan_aim_deadband_deg_ =
        this->declare_parameter<double>(
            "pan_aim_deadband_deg",
            2.0
        );

    pan_aim_hold_frames_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "pan_aim_hold_frames",
                14
            )
        );

    pan_aim_fov_margin_deg_ =
        this->declare_parameter<double>(
            "pan_aim_fov_margin_deg",
            5.0
        );

    // pan 회전축이 base_link 원점에서 떨어진 거리 [cm] (선언부
    // 헤더 주석 참고). pan=0 에서는 값이 얼마든 결과에 영향이
    // 없으므로 실측 전까지 0,0 으로 둔다.
    pan_pivot_offset_x_cm_ =
        this->declare_parameter<double>(
            "pan_pivot_offset_x_cm",
            0.0
        );

    pan_pivot_offset_y_cm_ =
        this->declare_parameter<double>(
            "pan_pivot_offset_y_cm",
            0.0
        );


    updateBevGeometry();


    // ============================================================
    // Log
    // ============================================================

    RCLCPP_INFO(
        this->get_logger(),
        "Kau Lane Detection Node started."
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Pipeline:"
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Camera Calibration"
        " -> Undistortion"
        " -> BEV"
        " -> HLS"
        " -> 3-Lane Sliding Window"
        " -> Centerline quintic Bezier"
    );


    // ============================================================
    // 경로 발행 (docs/경로_형식.md 9.9). QoS 는 CLAUDE.md §13.
    // ============================================================

    if (publish_path_)
    {
        lane_path_publisher_ =
            this->create_publisher<kau_msgs::msg::KauPath>(
                "/lane/center",
                rclcpp::QoS(1).reliable()
            );

        // 좌/우 흰선 경로. 하류가 corridor 물리 경계로 쓴다.
        // /lane/center 와 같은 QoS — 측위를 안 쓰는 구성에서는
        // 이 둘이 도로 경계의 유일한 근거다.
        lane_left_publisher_ =
            this->create_publisher<kau_msgs::msg::KauPath>(
                "/lane/left",
                rclcpp::QoS(1).reliable()
            );

        lane_right_publisher_ =
            this->create_publisher<kau_msgs::msg::KauPath>(
                "/lane/right",
                rclcpp::QoS(1).reliable()
            );

        viz_path_publisher_ =
            this->create_publisher<nav_msgs::msg::Path>(
                "/viz/path/lane",
                rclcpp::QoS(1).best_effort()
            );

        RCLCPP_INFO(
            this->get_logger(),
            "경로 발행: /lane/center (KauPath, degree=%d, frame_id=%s), "
            "/viz/path/lane (nav_msgs/Path, RViz 전용)",
            kau::bezier::DEGREE,
            path_frame_id_.c_str()
        );
    }
    else
    {
        RCLCPP_WARN(
            this->get_logger(),
            "publish_path=false. 제어점은 로그와 debug_image 로만 "
            "확인됩니다."
        );
    }
}
