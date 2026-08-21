// ====================================================================
// kau_lane_detection.cpp
//
// KauLaneDetectionNode 멤버 함수 구현.
// 클래스 선언은 include/kau_lane_detection/kau_lane_detection_node.hpp.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>




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
            std::bind(
                &KauLaneDetectionNode::imageCallback,
                this,
                std::placeholders::_1
            )
        );

    camera_info_subscriber_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",
            10,
            std::bind(
                &KauLaneDetectionNode::cameraInfoCallback,
                this,
                std::placeholders::_1
            )
        );


    // ============================================================
    // Publisher
    // ============================================================

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

    hls_binary_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/hls_binary",
            10
        );

    combined_binary_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/kau_lane_detection/combined_binary",
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


    // ============================================================
    // BEV Parameters
    //
    // ros2 param set 으로 실시간 튜닝 가능
    //
    //   src : 원본(undistort) 영상에서 잘라낼 사다리꼴
    //   dst : BEV 영상에서 그 사다리꼴이 놓일 사각형
    //
    // 사다리꼴 윗변 폭은 직접 주지 않고
    // 소실점(bev_vanishing_y)에서 자동으로 계산한다.
    //
    //   top_width = bottom_width
    //               * (top_y    - vanishing_y)
    //               / (bottom_y - vanishing_y)
    //
    // 이렇게 하면 top_y / bottom_y 를 바꿔도
    // 차선이 계속 평행하게 유지된다.
    //
    // bev_test.png 기준 실측값:
    //
    //   차선 간격 : y=270 에서 434px
    //               y=320 에서 683px (화면 밖)
    //
    //   -> bottom_y 는 좌/우 흰선이 화면 안에 들어오는
    //      270 근처가 상한
    //
    //   소실점: live undistort 프레임을 직접 변환해 스윕한
    //   결과 179.0 에서 BEV 위/아래 차선 간격이 일치
    //   (위 194.9 / 아래 195.4).
    //
    //   예전 값 184.7 은 bev_test.png 를 역변환해 복원한
    //   영상에서 뽑아서 편향돼 있었다. 그 값에서는 위쪽
    //   간격이 234px 로 아래(195px)보다 20% 넓었다.
    // ============================================================

    bev_src_center_x_ =
        this->declare_parameter<double>(
            "bev_src_center_x",
            240.0
        );

    // 윗변 y.
    //
    // 내릴수록 멀리 보고, BEV 하단 모서리의 검은 영역도 줄어든다.
    // (사다리꼴 윗변이 좁아지면서 원본 좌우 여분이 dst 바깥으로
    //  더 밀려나, 원본 밖을 샘플링하는 면적이 준다)
    //
    // 실측(live 프레임, margin 0.25, bottom_y 270):
    //
    //   top_y  검정%  차선간격  look-ahead  창검출 Y/L/R
    //     205   9.9%      195       3.5x    9/9 9/9 9/9
    //     195   5.2%      195       5.7x    9/9 9/9 9/9  <- 채택
    //     191   3.7%      196       7.6x    7/9 9/9 9/9
    //     185   1.7%      197      15.2x    5/9 8/9 7/9
    //
    // 차선 간격은 하단 스케일이 정하므로 top_y 와 무관하다.
    // 더 내리면 검정은 더 줄지만 원거리가 과하게 늘어나
    // 점선 노랑이 뭉개져 검출이 떨어진다.
    bev_src_top_y_ =
        this->declare_parameter<double>(
            "bev_src_top_y",
            195.0
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


    // ------------------------------------------------------------
    // Sliding Window margin
    //
    // 곡선 끊김의 지배적 원인이었다.
    //
    // 창 높이가 360/9 = 40px 이므로, 차선이 창 하나당
    // margin 보다 많이 옆으로 빠지면 그 창은 차선을 놓친다.
    //
    // 곡률(BEV 최상단 횡이동)별 창당 횡이동:
    //
    //   100px ->  19.8px
    //   140px ->  27.7px   <- margin 25 는 여기서 이미 실패
    //   180px ->  35.6px
    //   220px ->  43.5px
    //
    // 상한은 차선 간격(105px)의 절반인 52px.
    // 그 이상이면 옆 차선을 창 안에 물어버린다.
    //
    // 합성 곡선 검증 (곡률 260px, 3차선 평균 창검출률):
    //   margin 25 -> 70%
    //   margin 40 -> 89%
    // ------------------------------------------------------------

    window_margin_ =
        static_cast<int>(
            this->declare_parameter<int>(
                "window_margin",
                40
            )
        );


    // ------------------------------------------------------------
    // HLS 색 임계 (H, L, S)
    //
    // OpenCV BGR2HLS 규약: H 0~179, L/S 0~255.
    //
    // 노랑 S 하한이 중요하다. 실측(live BEV):
    //
    //   잔디          H 31~32, L 118~137, S 37~91
    //   노란 중앙선   H 18,    L 110,     S 225
    //
    // 예전 S 하한 70 은 잔디를 통과시켜서, 노랑 히스토그램
    // peak 가 실제 중앙선(x=239) 이 아니라 왼쪽 잔디(x=117)
    // 에 꽂혔다. 그러면 흰선 탐색 밴드도 같이 어긋나
    // 세 차선이 전부 무너진다.
    // ------------------------------------------------------------

    yellow_hls_lo_ =
        this->declare_parameter<std::vector<int64_t>>(
            "yellow_hls_lo",
            {15, 80, 150}
        );

    yellow_hls_hi_ =
        this->declare_parameter<std::vector<int64_t>>(
            "yellow_hls_hi",
            {35, 255, 255}
        );

    white_hls_lo_ =
        this->declare_parameter<std::vector<int64_t>>(
            "white_hls_lo",
            {0, 200, 0}
        );

    white_hls_hi_ =
        this->declare_parameter<std::vector<int64_t>>(
            "white_hls_hi",
            {180, 255, 70}
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
    // 방향성 추적 파라미터
    //
    // 창을 진행 방향으로 회전시키며 전진한다.
    // 이 넷이 "90도 코너에서 차선이 이어지는가" 를 결정한다.
    // ------------------------------------------------------------

    // 한 스텝 전진 거리 [BEV px].
    //
    // 코너 추종을 지배하는 값이다. turn 제한이나 min_pixels
    // 보다 이쪽이 훨씬 크게 작용한다.
    //
    // 창은 진행 방향으로 step, 횡으로 2*margin 이다. 즉 한
    // 스텝에서 흡수할 수 있는 방향 변화가 대략
    // atan(margin / step) 으로 정해진다.
    //
    //   step 40 -> 45도   step 25 -> 58도   step 20 -> 63도
    //
    // 90도 코너에는 점선 한 칸 사이에 방향이 70도 넘게 꺾이는
    // 지점이 있어서, step 40 이면 창이 차선을 지나쳐 허공으로
    // 나간다. 라이브 코너 프레임 실측: 노란선 대시 중심 11개가
    // (292,305) -> (277,294) 구간에서 -71도 -> -143도.
    //
    // 실패 77장 + 라이브 실측 (노란선 3스텝 이상 확보):
    //   step 40 miss 3 -> 40장,  라이브 코너 2스텝 (FAIL)
    //   step 30 miss 4 -> 48장
    //   step 25 miss 4 -> 55장,  라이브 코너 10스텝
    //   step 20 miss 5 -> 59장
    //
    // 중심선 적합 잔차 / 경로 생성률:
    //   step 40 : 65/77장  중앙  7.1px  90% 12.9  최대 20.4
    //   step 25 : 75/77장  중앙  9.7px  90% 15.7  최대 21.0
    //   step 20 : 77/77장  중앙 10.8px  90% 17.4  최대 31.9
    //
    // 20 은 성공률이 가장 높지만 잔차 꼬리가 길어진다. 25 가
    // 균형점.
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


    // ------------------------------------------------------------
    // 중심선 후보 이탈 판정 [lane_width_px 비율]
    //
    // 세 차선은 각자 중심선 후보를 하나씩 내놓는다. 그중 하나가
    // 엉뚱한 선을 물면 (노란선 소실 시 좌/우가 같은 흰선을 무는
    // 경우가 대표적) 가중평균이 통째로 끌려간다.
    //
    // 근거가 가장 많은 후보를 기준선으로 삼고, 거기서 이 비율
    // 이상 떨어진 후보는 버린다.
    //
    // 실패 65장 실측 (중심선 적합 잔차):
    //   제거 없음        중앙 10.4px  90%ile 77.1px  최대 105.5px
    //   0.30W 이상 제거  중앙  7.1px  90%ile 12.9px  최대  20.4px
    //   0.50W 이상 제거  중앙  8.5px  90%ile 16.0px  최대  32.3px
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // 흰선 쪽 판정 [lane_width_px 비율]
    //
    // 추적이 끝난 흰선이 노란선 기준으로 제 쪽에 있는지 본다.
    // 왼쪽 흰선은 -0.4W 보다 왼쪽, 오른쪽 흰선은 +0.4W 보다
    // 오른쪽이어야 한다.
    //
    // 실패 프레임(노란선 추적 성공 55장) 실측:
    //   검증 없음   L엉뚱 11장  R엉뚱 5장  같은선 8장
    //   부호 검증   L엉뚱  0장  R엉뚱 0장  같은선 0장
    //     (흰선 추적 성공은 L 40->29, R 48->43 으로 줄지만
    //      줄어든 만큼이 원래 반대쪽을 물던 것들이다)
    // ------------------------------------------------------------

    lane_side_min_ratio_ =
        this->declare_parameter<double>(
            "lane_side_min_ratio",
            0.40
        );


    center_outlier_ratio_ =
        this->declare_parameter<double>(
            "center_outlier_ratio",
            0.30
        );


    // ------------------------------------------------------------
    // 흰선 회랑 [lane_width 비율]
    //
    // 흰 마스크에는 차선만 있지 않다. 체커보드 연석, 정지선,
    // 급커브에서는 트랙 다른 구간의 차선까지 들어온다. 방향성
    // 창은 그것들을 구분하지 못하고 갈아탄다.
    //
    // 차로 폭은 물리적으로 고정이므로, 노란선에서 그만큼 떨어져
    // 있지 않은 점은 우리 차로의 흰선이 아니다. 이탈한 지점에서
    // 자른다 (통째로 버리면 정상이던 앞부분까지 잃는다).
    //
    // 실패 프레임 63장 실측:
    //   흰선 검출 L34/R53 중 이탈 L7/R3
    //   추적점 1309개 중 68개(5%) 절단, 3점 미만이 된 건 2건
    //   그 프레임들에서 중심선이 최대 117cm 달라졌다
    // ------------------------------------------------------------

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


    // ------------------------------------------------------------
    // 차로 폭 [BEV px] - 파라미터가 아니라 유도값
    //
    // 예전에는 lane_width_px 를 직접 넣었다. px 는 BEV 설정이
    // 바뀌면 같이 변하는 내부 단위라 다른 패키지와 맞출 수 없고,
    // 실제 값과 어긋나도 겉으로 드러나지 않는다.
    //
    // 이제 물리량에서 유도한다.
    //
    //   lane_width_px = lane_width_cm / sx
    //   sx            = camera_height_cm 과 intrinsic 에서 유도
    //
    // 설정 파일에는 자로 잴 수 있는 값만 남는다.
    // refreshDerivedScale() 이 CameraInfo 수신 시와
    // BEV 기하 변경 시 다시 계산한다.
    //
    // 아래 초기값은 CameraInfo 도착 전까지만 쓰이는 자리표시.
    // imageCallback 이 CameraInfo 없이는 조기 반환하므로
    // 검출에 실제로 쓰이는 일은 없다.
    // ------------------------------------------------------------

    lane_width_px_ = 100.0;


    // ------------------------------------------------------------
    // src 사다리꼴 오버레이 publish 여부
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // 히스토그램 밴드 높이 [BEV 하단 비율]
    //
    // 슬라이딩 윈도우 시작점을 찾는 밴드. 예전에는 0.40 이었다.
    //
    // 곡선에서 BEV 차선은 기울어져 있다. 밴드가 세로로 길면 그 안에서
    // 차선이 옆으로 쓸려 열 합산 peak 가 뭉개지고, 잔디나 연석에 얹힌다.
    //
    // 실패 70장 실측 (peak 와 실제 차선 위치의 오차):
    //
    //   밴드   중앙   90%ile   최대   margin초과   차로폭초과
    //   0.40     14      157    325      21장         9장
    //   0.20      9       53    260      11장         1장
    //   0.10      4       34     65       4장         0장
    //
    // "차로폭 초과" 는 아예 다른 차선을 잡았다는 뜻이다. 이때도
    // 슬라이딩 윈도우는 무언가를 따라가며 9/9 을 보고하므로,
    // 창 수만 보면 드러나지 않는다. 자신 있게 틀리는 실패다.
    //
    // 좁히면 점선 공백에 걸려 밴드가 빌 수 있어서, 비면 넓혀서
    // 재시도한다 (findHistogramPeak 참고).
    // ------------------------------------------------------------

    histogram_band_ratio_ =
        this->declare_parameter<double>(
            "histogram_band_ratio",
            0.10
        );


    publish_roi_overlay_ =
        this->declare_parameter<bool>(
            "publish_roi_overlay",
            true
        );


    // ============================================================
    // 경로 발행 (docs/경로_형식.md)
    // ============================================================

    // ------------------------------------------------------------
    // 발행 frame
    //
    // X 전방 / Y 좌 / 단위 cm.
    // ------------------------------------------------------------

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
    // 차선 정보 — 도로 모형
    //
    // lane_width_cm 만으로는 "차로 하나가 몇 cm 인가" 밖에 모른다.
    // 도로 단면에 선이 몇 개이고 각각 중앙선에서 얼마나 떨어져
    // 있는지는 road_map.hpp 의 kau_road::LINES 가 준다.
    //
    // 이 코드는 차로를 지정하지 않는다. 자차가 중앙선 왼쪽/오른쪽
    // 어느 차로에 있든 상관하지 않고 중앙선(황색 점선)을 그대로
    // 추종 목표로 삼는다. 그래서 여기엔 "지정 차선"이나 "통행
    // 방향" 파라미터가 없다 — road_map.hpp 에서 그 개념 자체를
    // 없앴다.
    //
    // 이 맵은 닫힌 루프 2차선이다. 흰선은 차선 경계가 아니라
    // 도로의 양 경계(트랙 바깥/안쪽)이고, 유일한 내부 경계가
    // 황색 점선 중앙선이다.
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
                kau_road::LINES[i].offset_cm,
                kau_road::LINES[i].name
            );

            cross += one;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "차선 정보 (road_map.hpp): 닫힌 루프 %d차선, "
            "차선 폭 %.2fcm, 도로 폭 %.2fcm, 차로 조건 없이 "
            "중앙선 추종 | 단면 %s",
            kau_road::LANE_COUNT,
            kau_road::LANE_WIDTH_CM,
            kau_road::ROAD_WIDTH_CM,
            cross.c_str()
        );
    }


    // 근거(노란선 또는 흰선 두 개)가 끊겨도 직전 중앙선 대비
    // 위치 판정을 유지할 프레임 수. 점선 공백과 순간 가림 대응.
    center_hold_frames_ =
        this->declare_parameter<int>(
            "center_hold_frames",
            10
        );


    // ============================================================
    // 발행 게이트
    //
    // 맵 1바퀴(1546 프레임) 실측에서 나온 문제:
    // 근거가 한쪽 차선 3창뿐인데도 곡률반경 45cm 경로를 발행했다.
    // 휠베이스 18cm / 최대조향 20도면 최소 회전반경이 49.5cm라
    // 물리적으로 불가능한 명령이 컨트롤러로 나간 것이다.
    //
    // 창 3개로 2차 적합하면 자유도 0, 잔차 0인 완전 결정계라
    // 아무리 엉터리여도 "완벽하게 맞는" 곡선이 나온다.
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
    max_lateral_cm_ =
        this->declare_parameter<double>(
            "max_lateral_cm",
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
    // 경로 발행 (docs/경로_형식.md 9.9)
    //
    //   /lane/center   KauPath        RELIABLE, depth 1, 10Hz
    //   /viz/path/lane nav_msgs/Path  BEST_EFFORT, RViz 전용
    //
    // /lane/center 는 localization 상실 시 유일한 경로 source 라
    // RELIABLE 이다. 이산화는 viz 쪽에서만 일어난다.
    // ============================================================

    if (publish_path_)
    {
        lane_path_publisher_ =
            this->create_publisher<kau_msgs::msg::KauPath>(
                "/lane/center",
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




// ================================================================
// 가중 최소제곱 다항식 적합  v = f(t)
//
// 중심선을 매개변수 s (누적 현길이) 로 적합하기 위한 도구다.
// x(s), y(s) 를 각각 한 번씩 부른다.
//
// 예전에는 x = f(y) 하나만 적합했다. 그 표현은 차선이 가로로
// 누우면 dx/dy = 무한대라 어떤 다항식으로도 표현할 수 없다.
// 90도 코너에서 차선이 끊기던 근본 원인이 여기였다.
//
// s 로 매개화하면 방향에 무관해진다. x(s) 도 y(s) 도 완만한
// 다항식이고, 코너에서도 발산하지 않는다.
//
// OpenCV 에 polyfit 이 없어 Vandermonde 행렬을 만들어
// cv::solve(DECOMP_QR) 로 직접 푼다. 가중치는 각 행에
// sqrt(w) 를 곱해서 넣는다 (가중 최소제곱의 표준 변형).
//
// coeffs[0]*t^order + ... + coeffs[order]
// ================================================================

bool KauLaneDetectionNode::polyFitW(
    const std::vector<double> & ts,
    const std::vector<double> & vs,
    const std::vector<double> & weights,
    int order,
    std::vector<double> & coeffs)
{
    const int n =
        static_cast<int>(ts.size());


    if (
        order < 1 ||
        n < order + 1 ||
        n != static_cast<int>(vs.size())
    )
    {
        return false;
    }


    const bool use_w =
        weights.size() == ts.size();


    // t 범위가 없으면 (모든 점이 같은 자리) 적합 불가
    const auto t_range =
        std::minmax_element(
            ts.begin(),
            ts.end()
        );


    if (*t_range.second - *t_range.first < 1e-6)
    {
        return false;
    }


    cv::Mat a(
        n,
        order + 1,
        CV_64F
    );

    cv::Mat b(
        n,
        1,
        CV_64F
    );


    for (
        int i = 0;
        i < n;
        ++i
    )
    {
        const double rw =
            std::sqrt(
                use_w ? std::max(0.0, weights[i]) : 1.0
            );


        double power = 1.0;


        for (
            int j = order;
            j >= 0;
            --j
        )
        {
            a.at<double>(i, j) = rw * power;

            power *= ts[i];
        }


        b.at<double>(i, 0) = rw * vs[i];
    }


    cv::Mat solution;


    if (
        !cv::solve(
            a,
            b,
            solution,
            cv::DECOMP_QR
        )
    )
    {
        return false;
    }


    coeffs.assign(
        order + 1,
        0.0
    );


    for (
        int j = 0;
        j <= order;
        ++j
    )
    {
        coeffs[j] = solution.at<double>(j, 0);
    }


    return true;
}




// ================================================================
// 폴리라인 위로 투영
//
// 세 차선 후보를 하나의 매개변수 s 로 묶기 위한 도구다.
//
// 후보끼리 s 를 각자 0 부터 세면 안 된다. 세 차선은 서로 다른
// 지점에서 추적이 시작되므로 같은 s 가 같은 위치를 뜻하지
// 않는다. 실패 65장 실측 잔차 중앙값이 24.6px 였고, 그 대부분이
// 이 어긋남에서 나왔다 (기준선 투영 시 10.4px).
//
// 양 끝 구간에서는 t 를 [0,1] 로 자르지 않는다. 기준선보다
// 앞/뒤로 나간 후보 점을 끝에 뭉치게 두면 그 자리만 과대
// 가중되기 때문이다. s < 0 이 나올 수 있고, 호출측에서
// 전체를 s_min 만큼 평행이동해 [0, L] 로 되돌린다.
// ================================================================

void KauLaneDetectionNode::projectOnPolyline(
    const std::vector<cv::Point2d> & poly,
    const std::vector<double> & cum,
    const cv::Point2d & p,
    double & s_out,
    double & d_out)
{
    s_out = 0.0;

    d_out = std::numeric_limits<double>::max();


    const int nseg =
        static_cast<int>(poly.size()) - 1;


    for (
        int j = 0;
        j < nseg;
        ++j
    )
    {
        const double dx = poly[j + 1].x - poly[j].x;

        const double dy = poly[j + 1].y - poly[j].y;

        const double len2 = dx * dx + dy * dy;


        if (len2 < 1e-12)
        {
            continue;
        }


        double t =
            ((p.x - poly[j].x) * dx +
             (p.y - poly[j].y) * dy) / len2;


        // 양 끝 구간만 외삽 허용
        const double lo = (j == 0)        ? -1e9 : 0.0;

        const double hi = (j == nseg - 1) ?  1e9 : 1.0;

        t = std::clamp(t, lo, hi);


        const double qx = poly[j].x + t * dx;

        const double qy = poly[j].y + t * dy;

        const double d =
            std::hypot(p.x - qx, p.y - qy);


        if (d < d_out)
        {
            d_out = d;

            s_out = cum[j] + t * std::sqrt(len2);
        }
    }
}




// ================================================================
// 기준선 대비 부호 있는 횡거리의 중앙값
//
// "왼쪽 흰선이 정말 왼쪽에 있는가" 를 확인하는 데 쓴다.
//
// 히스토그램 base 탐색은 x 구간
//
//   왼쪽  [yellow - 1.6W, yellow - 0.4W]
//   오른쪽 [yellow + 0.4W, yellow + 1.6W]
//
// 으로 되어 있다. 차선이 세로로 서 있을 때만 "x 로 왼쪽" 이
// "차로의 왼쪽" 과 같다. 90도 코너에서 차선이 가로로 누우면
// 좌/우 흰선은 노란선의 위/아래에 쌓이므로, x 구간 탐색이
// 반대쪽 선이나 같은 선을 물어 온다.
//
// 실패 프레임 중 노란선 추적이 된 55장 실측:
//   왼쪽 흰선이 왼쪽에 없음      11장
//   오른쪽 흰선이 오른쪽에 없음   5장
//   좌/우가 같은 선을 물음        8장
//
// base 탐색 자체를 법선 방향으로 바꿔 봐도 검출 수만 크게
// 줄고 (L 40 -> 17장) 이득이 없었다. 그래서 탐색은 그대로 두고,
// 추적 결과를 노란선 기준으로 검증해 반대쪽이면 버린다.
// ================================================================

std::vector<double> KauLaneDetectionNode::lateralOffsets(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts)
{
    std::vector<double> offs;


    if (ref.size() < 2 || pts.empty())
    {
        return offs;
    }


    offs.reserve(pts.size());


    for (const cv::Point2d & p : pts)
    {
        double best_d = std::numeric_limits<double>::max();

        double best_o = 0.0;


        for (
            std::size_t j = 0;
            j + 1 < ref.size();
            ++j
        )
        {
            const double dx = ref[j + 1].x - ref[j].x;

            const double dy = ref[j + 1].y - ref[j].y;

            const double len2 = dx * dx + dy * dy;


            if (len2 < 1e-12)
            {
                continue;
            }


            const double t =
                std::clamp(
                    ((p.x - ref[j].x) * dx +
                     (p.y - ref[j].y) * dy) / len2,
                    0.0,
                    1.0
                );

            const double qx = ref[j].x + t * dx;

            const double qy = ref[j].y + t * dy;

            const double d = std::hypot(p.x - qx, p.y - qy);


            if (d < best_d)
            {
                const double len = std::sqrt(len2);

                // 진행방향 +90도 = 차량 오른쪽
                best_o =
                    (p.x - qx) * (-dy / len) +
                    (p.y - qy) * ( dx / len);

                best_d = d;
            }
        }


        offs.push_back(best_o);
    }


    return offs;
}




// ================================================================
// 부호 있는 횡거리의 중앙값
// ================================================================

double KauLaneDetectionNode::medianLateralOffset(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts)
{
    std::vector<double> offs =
        lateralOffsets(ref, pts);


    if (offs.empty())
    {
        return 0.0;
    }


    std::nth_element(
        offs.begin(),
        offs.begin() + offs.size() / 2,
        offs.end()
    );


    return offs[offs.size() / 2];
}




// ================================================================
// 기준선 대비 부호 있는 횡거리 하나 (외삽 허용)
//
// lateralOffsets 는 각 구간의 투영 매개변수 t 를 [0,1] 로 자른다.
// 추적점끼리 재는 데에는 맞다. 하지만 자차 위치는 추적 시작점
// 보다 뒤(BEV 아래쪽)에 있어서, 자르면 첫 점까지의 직선거리가
// 나와 값이 부풀려지고 방향도 어긋난다.
//
// 양 끝 구간에서만 t 를 밖으로 열어 둔다. 중간 구간까지 열면
// 폴리라인이 꺾이는 곳에서 엉뚱한 구간에 붙는다.
// ================================================================

double KauLaneDetectionNode::lateralOffsetAt(
    const std::vector<cv::Point2d> & ref,
    const cv::Point2d & p)
{
    if (ref.size() < 2)
    {
        return 0.0;
    }


    const std::size_t nseg = ref.size() - 1;

    double best_d = std::numeric_limits<double>::max();

    double best_o = 0.0;


    for (
        std::size_t j = 0;
        j < nseg;
        ++j
    )
    {
        const double dx = ref[j + 1].x - ref[j].x;

        const double dy = ref[j + 1].y - ref[j].y;

        const double len2 = dx * dx + dy * dy;


        if (len2 < 1e-12)
        {
            continue;
        }


        const double lo = (j == 0)        ? -1e9 : 0.0;

        const double hi = (j == nseg - 1) ?  1e9 : 1.0;


        const double t =
            std::clamp(
                ((p.x - ref[j].x) * dx +
                 (p.y - ref[j].y) * dy) / len2,
                lo,
                hi
            );

        const double qx = ref[j].x + t * dx;

        const double qy = ref[j].y + t * dy;

        const double d = std::hypot(p.x - qx, p.y - qy);


        if (d < best_d)
        {
            const double len = std::sqrt(len2);

            // 진행방향 +90도 = 차량 오른쪽
            best_o =
                (p.x - qx) * (-dy / len) +
                (p.y - qy) * ( dx / len);

            best_d = d;
        }
    }


    return best_o;
}




// ================================================================
// 회랑 이탈 지점에서 절단
//
// 흰 마스크에는 차선만 있는 게 아니다. 체커보드 연석, 정지선,
// 그리고 급커브에서는 트랙의 다른 구간 차선까지 들어온다.
// 방향성 창은 그것들을 구분하지 못하고 그대로 갈아탄다.
//
// 실측 (실패 프레임 63장, 노란선 추적 성공분):
//   흰선 검출 L 34 / R 53 중 회랑을 벗어난 것 L 7 / R 3
//   추적점 1309개 중 68개(5%)만 잘리고, 잘린 뒤 3점 미만은 2건
//   그 7 프레임에서 중심선이 최대 117cm 달라졌다
//
// 차로 폭은 물리적으로 고정이므로, 노란선에서 그만큼 떨어져
// 있지 않은 점은 우리 차로의 흰선이 아니다. 통째로 버리면
// 정상이던 앞부분까지 잃으므로 이탈 지점에서 자른다.
// ================================================================

std::vector<cv::Point2d> KauLaneDetectionNode::truncateAtCorridor(
    const std::vector<cv::Point2d> & track,
    const std::vector<cv::Point2d> & ref,
    double want_sign,
    double lo_px,
    double hi_px)
{
    if (ref.size() < 2 || track.empty())
    {
        return track;
    }


    const std::vector<double> offs =
        lateralOffsets(ref, track);


    if (offs.size() != track.size())
    {
        return track;
    }


    std::size_t keep = 0;


    while (keep < track.size())
    {
        const double o = offs[keep] * want_sign;

        if (o < lo_px || o > hi_px)
        {
            break;
        }

        ++keep;
    }


    return std::vector<cv::Point2d>(
        track.begin(),
        track.begin() + static_cast<long>(keep)
    );
}




// ================================================================
// 점열을 국소 법선 방향으로 평행이동
//
// 흰선에서 중심선을 만들 때 쓴다.
//
// 예전에는 x 좌표에 +- lane_width_px 를 더했다. 차선이 세로로
// 서 있을 때만 맞는 식이다. 90도 코너에서 차선이 가로로 누우면
// 중심선은 x 가 아니라 y 로 밀려야 하므로 완전히 빗나간다.
//
// 국소 접선 t 를 +90도 회전한 법선
//
//   t = (0,-1) (화면 위 = 차량 전방)  ->  n = (1,0) (차량 오른쪽)
//
// 을 쓰면 방향에 무관하게 항상 옳다.
// ================================================================

std::vector<cv::Point2d> KauLaneDetectionNode::offsetTrack(
    const std::vector<cv::Point2d> & pts,
    double offset_px)
{
    std::vector<cv::Point2d> out;


    const std::size_t n = pts.size();


    if (n < 2)
    {
        return out;
    }


    out.reserve(n);


    for (
        std::size_t i = 0;
        i < n;
        ++i
    )
    {
        // 중앙차분. 끝점은 한쪽 차분.
        const std::size_t a = (i == 0) ? 0 : i - 1;

        const std::size_t b = (i + 1 < n) ? i + 1 : i;


        double tx = pts[b].x - pts[a].x;

        double ty = pts[b].y - pts[a].y;


        const double len = std::hypot(tx, ty);


        if (len < 1e-9)
        {
            tx = 0.0;

            ty = -1.0;
        }
        else
        {
            tx /= len;

            ty /= len;
        }


        out.push_back(
            cv::Point2d(
                pts[i].x + offset_px * (-ty),
                pts[i].y + offset_px * ( tx)
            )
        );
    }


    return out;
}




// ================================================================
// 파라미터(int 배열) -> cv::Scalar
// ================================================================

cv::Scalar KauLaneDetectionNode::toScalar(
    const std::vector<int64_t> & v)
{
    return cv::Scalar(
        v.size() > 0 ? static_cast<double>(v[0]) : 0.0,
        v.size() > 1 ? static_cast<double>(v[1]) : 0.0,
        v.size() > 2 ? static_cast<double>(v[2]) : 0.0
    );
}




double KauLaneDetectionNode::polyEval(
    const std::vector<double> & coeffs,
    double y)
{
    double value = 0.0;


    for (const double coefficient : coeffs)
    {
        value = value * y + coefficient;
    }


    return value;
}




// ================================================================
// BEV Geometry
//
// src 사다리꼴 -> dst 사각형 변환 행렬을 만든다.
//
// 핵심:
//
//   src의 좌/우 변을 위로 연장했을 때 만나는 점(소실점)이
//   실제 도로 차선의 소실점과 같아야
//   BEV에서 차선이 "평행한 수직선"이 된다.
//
//   그래서 윗변 폭을 직접 주는 대신
//   소실점에서 역산한다.
// ================================================================

void KauLaneDetectionNode::updateBevGeometry()
{
    // ------------------------------------------------------------
    // src 사다리꼴
    // ------------------------------------------------------------

    const float center_x =
        static_cast<float>(
            bev_src_center_x_
        );

    const float half_bottom =
        static_cast<float>(
            bev_src_bottom_width_ * 0.5
        );

    const float top_y =
        static_cast<float>(
            bev_src_top_y_
        );

    const float bottom_y =
        static_cast<float>(
            bev_src_bottom_y_
        );


    // ------------------------------------------------------------
    // 윗변 폭 = 소실점을 지나도록 역산
    //
    //   half_top = half_bottom
    //              * (top_y    - vanishing_y)
    //              / (bottom_y - vanishing_y)
    // ------------------------------------------------------------

    const double depth_bottom =
        bev_src_bottom_y_ - bev_vanishing_y_;

    const double depth_top =
        bev_src_top_y_ - bev_vanishing_y_;


    if (
        depth_bottom <= 1.0 ||
        depth_top <= 1.0
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid BEV geometry: "
            "bev_vanishing_y (%.1f) must be above "
            "bev_src_top_y (%.1f) and "
            "bev_src_bottom_y (%.1f). "
            "Keeping previous transform.",
            bev_vanishing_y_,
            bev_src_top_y_,
            bev_src_bottom_y_
        );

        return;
    }


    const float half_top =
        static_cast<float>(
            half_bottom *
            depth_top /
            depth_bottom
        );


    bev_src_top_width_ =
        2.0 *
        static_cast<double>(
            half_top
        );


    bev_src_points_ =
    {
        cv::Point2f(
            center_x - half_top,
            top_y
        ),                                  // top-left

        cv::Point2f(
            center_x + half_top,
            top_y
        ),                                  // top-right

        cv::Point2f(
            center_x + half_bottom,
            bottom_y
        ),                                  // bottom-right

        cv::Point2f(
            center_x - half_bottom,
            bottom_y
        )                                   // bottom-left
    };


    // ------------------------------------------------------------
    // dst 사각형
    //
    // 좌우에 여백을 주어 차선이 화면 끝에 붙지 않고
    // 가운데로 모이게 한다.
    // ------------------------------------------------------------

    const float out_w =
        static_cast<float>(
            bev_out_width_
        );

    const float out_h =
        static_cast<float>(
            bev_out_height_
        );

    const float margin =
        out_w *
        static_cast<float>(
            std::clamp(
                bev_dst_margin_ratio_,
                0.0,
                0.45
            )
        );

    const float x_left = margin;

    const float x_right = out_w - margin;


    bev_dst_points_ =
    {
        cv::Point2f(
            x_left,
            0.0f
        ),

        cv::Point2f(
            x_right,
            0.0f
        ),

        cv::Point2f(
            x_right,
            out_h
        ),

        cv::Point2f(
            x_left,
            out_h
        )
    };


    perspective_matrix_ =
        cv::getPerspectiveTransform(
            bev_src_points_,
            bev_dst_points_
        );


    RCLCPP_INFO(
        this->get_logger(),
        "BEV src: top(%.1f ~ %.1f @ y=%.1f, width %.1f) "
        "bottom(%.1f ~ %.1f @ y=%.1f, width %.1f)",
        center_x - half_top,
        center_x + half_top,
        top_y,
        bev_src_top_width_,
        center_x - half_bottom,
        center_x + half_bottom,
        bottom_y,
        bev_src_bottom_width_
    );

    RCLCPP_INFO(
        this->get_logger(),
        "BEV vanishing y = %.1f "
        "-> look-ahead ratio %.1fx",
        bev_vanishing_y_,
        depth_bottom / depth_top
    );

    RCLCPP_INFO(
        this->get_logger(),
        "BEV dst: x %.1f ~ %.1f "
        "(margin ratio %.2f) size %d x %d",
        x_left,
        x_right,
        bev_dst_margin_ratio_,
        bev_out_width_,
        bev_out_height_
    );
}




// ================================================================
// Parameter Refresh
//
// ros2 param set 으로 값이 바뀌면 변환 행렬을 다시 만든다.
// ================================================================

void KauLaneDetectionNode::refreshBevParameters()
{
    const double center_x =
        this->get_parameter(
            "bev_src_center_x"
        ).as_double();

    const double top_y =
        this->get_parameter(
            "bev_src_top_y"
        ).as_double();

    const double bottom_y =
        this->get_parameter(
            "bev_src_bottom_y"
        ).as_double();

    const double vanishing_y =
        this->get_parameter(
            "bev_vanishing_y"
        ).as_double();

    const double bottom_width =
        this->get_parameter(
            "bev_src_bottom_width"
        ).as_double();

    const double margin_ratio =
        this->get_parameter(
            "bev_dst_margin_ratio"
        ).as_double();

    // 축척을 정하는 물리 파라미터. 바뀌면 lane_width_px 도 따라간다.
    const double cam_height =
        this->get_parameter(
            "camera_height_cm"
        ).as_double();

    const double lane_width =
        this->get_parameter(
            "lane_width_cm"
        ).as_double();

    const int out_width =
        static_cast<int>(
            this->get_parameter(
                "bev_out_width"
            ).as_int()
        );

    const int out_height =
        static_cast<int>(
            this->get_parameter(
                "bev_out_height"
            ).as_int()
        );


    window_margin_ =
        static_cast<int>(
            this->get_parameter(
                "window_margin"
            ).as_int()
        );

    publish_roi_overlay_ =
        this->get_parameter(
            "publish_roi_overlay"
        ).as_bool();


    const bool changed =
        center_x      != bev_src_center_x_    ||
        top_y         != bev_src_top_y_       ||
        bottom_y      != bev_src_bottom_y_    ||
        vanishing_y   != bev_vanishing_y_     ||
        bottom_width  != bev_src_bottom_width_ ||
        margin_ratio  != bev_dst_margin_ratio_ ||
        out_width     != bev_out_width_       ||
        out_height    != bev_out_height_      ||
        cam_height    != camera_height_cm_    ||
        lane_width    != lane_width_cm_;


    if (!changed)
    {
        return;
    }


    bev_src_center_x_ = center_x;

    bev_src_top_y_ = top_y;

    bev_src_bottom_y_ = bottom_y;

    bev_vanishing_y_ = vanishing_y;

    bev_src_bottom_width_ = bottom_width;

    bev_dst_margin_ratio_ = margin_ratio;

    bev_out_width_ = out_width;

    bev_out_height_ = out_height;

    camera_height_cm_ = cam_height;

    lane_width_cm_ = lane_width;


    updateBevGeometry();

    // 사다리꼴이나 축척이 바뀌었으므로 lane_width_px 를 다시 유도
    refreshDerivedScale();
}




// ================================================================
// CameraInfo Callback
// ================================================================

void KauLaneDetectionNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // ------------------------------------------------------------
    // Camera Matrix K
    // ------------------------------------------------------------

    camera_matrix_ = cv::Mat(
        3,
        3,
        CV_64F,
        const_cast<double*>(msg->k.data())
    ).clone();


    // ------------------------------------------------------------
    // Distortion Coefficients
    // ------------------------------------------------------------

    distortion_coefficients_ = cv::Mat(
        static_cast<int>(msg->d.size()),
        1,
        CV_64F,
        const_cast<double*>(msg->d.data())
    ).clone();


    camera_calibrated_ = true;


    // ------------------------------------------------------------
    // 축척이 intrinsic 에 의존하므로 유도값을 다시 계산한다
    // ------------------------------------------------------------

    refreshDerivedScale();


    // ------------------------------------------------------------
    // Log
    // ------------------------------------------------------------

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "CameraInfo received."
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Image size: %u x %u",
        msg->width,
        msg->height
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Distortion model: %s",
        msg->distortion_model.c_str()
    );


    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Camera Matrix K:"
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(0, 0),
        camera_matrix_.at<double>(0, 1),
        camera_matrix_.at<double>(0, 2)
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(1, 0),
        camera_matrix_.at<double>(1, 1),
        camera_matrix_.at<double>(1, 2)
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(2, 0),
        camera_matrix_.at<double>(2, 1),
        camera_matrix_.at<double>(2, 2)
    );


    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Distortion coefficients: %zu values",
        msg->d.size()
    );
}




// ================================================================
// Histogram Peak
//
// 첨부 코드의 방식:
//
// roi_hist = edges.rowRange(height * 0.6, height)
//
// 즉 BEV 하단 40%만 이용해서 histogram을 계산
// ================================================================

int KauLaneDetectionNode::findHistogramPeak(
    const cv::Mat & mask,
    int x_start,
    int x_end,
    double * band_used)
{
    x_start = std::max(
        0,
        x_start
    );

    x_end = std::min(
        mask.cols,
        x_end
    );


    if (band_used != nullptr)
    {
        *band_used = 0.0;
    }


    if (x_start >= x_end || mask.rows <= 0)
    {
        return -1;
    }


    // ------------------------------------------------------------
    // 좁은 밴드부터 시도하고, 비면 넓혀서 재시도.
    //
    // 노란 중앙선이 점선이라 좁은 밴드가 통째로 점선 공백에 걸릴 수
    // 있다. 그때는 넓혀서라도 찾는 편이 아무것도 못 찾는 것보다 낫다.
    // 대신 넓힌 밴드의 peak 는 곡선에서 부정확하므로, 어느 밴드가
    // 쓰였는지 호출측에 알려 준다.
    // ------------------------------------------------------------

    const double base =
        std::clamp(
            histogram_band_ratio_,
            0.02,
            1.0
        );

    const double ratios[3] =
    {
        base,
        std::min(base * 2.0, 1.0),
        std::min(base * 4.0, 1.0)
    };


    for (const double ratio : ratios)
    {
        const int start_y =
            std::clamp(
                static_cast<int>(mask.rows * (1.0 - ratio)),
                0,
                mask.rows - 1
            );


        std::vector<int> histogram(
            mask.cols,
            0
        );


        for (
            int y = start_y;
            y < mask.rows;
            ++y
        )
        {
            const uchar * row = mask.ptr<uchar>(y);

            for (
                int x = x_start;
                x < x_end;
                ++x
            )
            {
                if (row[x] > 0)
                {
                    histogram[x]++;
                }
            }
        }


        int max_x = -1;

        int max_value = 0;


        for (
            int x = x_start;
            x < x_end;
            ++x
        )
        {
            if (histogram[x] > max_value)
            {
                max_value = histogram[x];

                max_x = x;
            }
        }


        if (max_value > 0)
        {
            if (band_used != nullptr)
            {
                *band_used = ratio;
            }

            return max_x;
        }
    }


    return -1;
}




// ================================================================
// 방향성 슬라이딩 윈도우
//
// 예전 방식은 창을 y 격자에 못 박아 두고 아래에서 위로만
// 올렸다. 그 구조에는 세 가지 한계가 동시에 있었다.
//
//   1. 결과가 x = f(y) 였다.
//      차선이 가로로 누우면 dx/dy = 무한대라 표현 불가.
//   2. 가로 차선은 y 밴드 하나(360/9 = 40px)에 통째로 들어간다.
//      잡히는 창이 1개뿐이라 MIN_VALID_WINDOWS(3) 를 못 넘긴다.
//   3. 가로 구간의 창 무게중심은 창 중앙 그대로다.
//      창이 옆으로 따라가질 못하니 다음 창은 그냥 빈 하늘이다.
//
// 그래서 90도 코너에서 차선이 통째로 끊겼다.
//
// 지금 방식은 창을 진행 방향 theta 로 회전시키고, 그 방향으로
// 한 스텝씩 전진한다.
//
//   - 창 안 픽셀의 무게중심으로 위치를 보정
//   - 직전 점 -> 새 무게중심 벡터로 theta 갱신
//     (한 스텝 회전량은 track_max_turn_deg 로 제한 — 되꺾임 방지)
//   - 비면 관성으로 직진 (점선 공백 통과), 연속 miss 가
//     track_max_miss 를 넘으면 종료
//
// 결과는 y 격자에 묶이지 않은 "순서 있는 점열"이다.
// 방향에 아무 전제가 없으므로 90도든 180도든 따라간다.
// ================================================================

KauLaneDetectionNode::LaneDetectionResult KauLaneDetectionNode::detectLaneSlidingWindow(
    const cv::Mat & mask,
    int base_x,
    cv::Mat & debug_image,
    const cv::Scalar & window_color)
{
    LaneDetectionResult result;


    if (base_x < 0)
    {
        return result;
    }


    const int height = mask.rows;

    const int width = mask.cols;


    // 진행 방향 창 길이 = 한 스텝 전진 거리.
    //
    // 예전에는 height / NUM_WINDOWS (= 40px) 에 묶여 있었다.
    // 그 값으로는 90도 코너의 급전환 지점에서 창이 차선을
    // 지나쳐 버린다 (파라미터 선언부 주석 참고).
    const double step =
        std::max(4.0, track_step_px_);

    const double half = 0.5 * step;

    const double margin =
        static_cast<double>(window_margin_);

    const double max_turn =
        track_max_turn_deg_ * M_PI / 180.0;


    // 창을 완전히 감싸는 반경 (축정렬 bounding box 용)
    const double reach =
        std::hypot(half, margin);


    double cx = static_cast<double>(base_x);

    double cy = static_cast<double>(height) - half;

    // 화면 위(-y) = 차량 전방
    double th = -M_PI / 2.0;

    int miss = 0;


    for (
        int s = 0;
        s < track_max_steps_;
        ++s
    )
    {
        const double ct = std::cos(th);

        const double st = std::sin(th);


        const int x0 =
            std::max(0, static_cast<int>(std::floor(cx - reach)));

        const int x1 =
            std::min(width, static_cast<int>(std::ceil(cx + reach)) + 1);

        const int y0 =
            std::max(0, static_cast<int>(std::floor(cy - reach)));

        const int y1 =
            std::min(height, static_cast<int>(std::ceil(cy + reach)) + 1);


        // --------------------------------------------------------
        // 회전 창 내부 픽셀의 무게중심
        //
        //   u =  dx cos + dy sin   진행 방향  (|u| <= half)
        //   v = -dx sin + dy cos   횡 방향    (|v| <= margin)
        // --------------------------------------------------------

        double sum_x = 0.0;

        double sum_y = 0.0;

        int pixel_count = 0;


        for (
            int y = y0;
            y < y1;
            ++y
        )
        {
            const uchar * row = mask.ptr<uchar>(y);

            const double dy =
                static_cast<double>(y) - cy;


            for (
                int x = x0;
                x < x1;
                ++x
            )
            {
                if (row[x] == 0)
                {
                    continue;
                }


                const double dx =
                    static_cast<double>(x) - cx;

                const double u =  dx * ct + dy * st;

                const double v = -dx * st + dy * ct;


                if (
                    std::abs(u) <= half &&
                    std::abs(v) <= margin
                )
                {
                    sum_x += x;

                    sum_y += y;

                    pixel_count++;
                }
            }
        }


        const bool found =
            pixel_count > min_pixels_;


        if (found)
        {
            const double mx = sum_x / pixel_count;

            const double my = sum_y / pixel_count;


            // ----------------------------------------------------
            // 진행 방향 갱신
            //
            // 회전량을 제한하지 않으면 옆 차선이나 노이즈를
            // 물었을 때 창이 되꺾여 왔던 길을 되돌아간다.
            // ----------------------------------------------------

            if (!result.track_px.empty())
            {
                const cv::Point2d & prev =
                    result.track_px.back();

                const double vx = mx - prev.x;

                const double vy = my - prev.y;


                if (std::hypot(vx, vy) > 1.0)
                {
                    const double target =
                        std::atan2(vy, vx);

                    const double delta =
                        std::remainder(
                            target - th,
                            2.0 * M_PI
                        );


                    th +=
                        std::clamp(
                            delta,
                            -max_turn,
                            max_turn
                        );
                }
            }


            cx = mx;

            cy = my;

            result.track_px.push_back(
                cv::Point2d(cx, cy)
            );

            result.found_count++;

            miss = 0;


            // ----------------------------------------------------
            // 시각화: 실제로 잡은 창만 그린다.
            // 회전 사각형이라 차선 방향이 눈으로 바로 보인다.
            // ----------------------------------------------------

            cv::Point2f corners[4];

            cv::RotatedRect(
                cv::Point2f(
                    static_cast<float>(cx),
                    static_cast<float>(cy)
                ),
                cv::Size2f(
                    static_cast<float>(step),
                    static_cast<float>(2.0 * margin)
                ),
                static_cast<float>(th * 180.0 / M_PI)
            ).points(corners);


            for (
                int i = 0;
                i < 4;
                ++i
            )
            {
                cv::line(
                    debug_image,
                    corners[i],
                    corners[(i + 1) % 4],
                    window_color,
                    1
                );
            }
        }
        else
        {
            miss++;


            if (miss > track_max_miss_)
            {
                break;
            }
        }


        // --------------------------------------------------------
        // 다음 창으로 전진 (방향은 theta)
        // --------------------------------------------------------

        cx += step * std::cos(th);

        cy += step * std::sin(th);


        if (
            cx < -margin ||
            cx > width + margin ||
            cy < -half ||
            cy > height + half
        )
        {
            break;
        }
    }


    result.detected =
        result.found_count >= MIN_VALID_WINDOWS;

    result.valid = result.detected;


    if (!result.valid)
    {
        return result;
    }


    // ============================================================
    // 추적 궤적 시각화
    //
    // 다항식으로 다시 그리지 않는다. 점열 자체가 결과다.
    // ============================================================

    for (
        std::size_t i = 1;
        i < result.track_px.size();
        ++i
    )
    {
        cv::line(
            debug_image,
            cv::Point(
                static_cast<int>(result.track_px[i - 1].x),
                static_cast<int>(result.track_px[i - 1].y)
            ),
            cv::Point(
                static_cast<int>(result.track_px[i].x),
                static_cast<int>(result.track_px[i].y)
            ),
            window_color,
            2
        );
    }


    return result;
}




// ================================================================
// BEV 축척 [cm/px]
//
// 가로 축척은 실 차로 폭으로 정한다.
//
//   cm_per_px_x = lane_width_cm / lane_width_px
//
// 세로 축척은 따로 잴 필요가 없다. 카메라 intrinsic 과
// 소실점만으로 세로/가로 비가 결정되고, 카메라 높이 h 는
// 분자 분모에서 약분되기 때문이다.
//
// 유도 (카메라 좌표 x 우 / y 아래 / z 전방, roll 0):
//
//   a    = (v - cy) / fy                 행 v 의 정규화 y
//   a_h  = (vanishing_y - cy) / fy       지평선
//
//   지평선 광선은 지면과 평행하므로 지면 법선은
//   n = (0, -1, a_h) / sqrt(1 + a_h^2).
//
//   광선 P = t (b, a, 1) 이 지면(n·P = -h)과 만나는 점:
//
//     t(a)/h = sqrt(1 + a_h^2) / (a - a_h)        광축 depth
//     D(a)/h = (1 + a a_h)   / (a - a_h)          지면 전방거리
//
//   src 사다리꼴의 좌/우 변은 소실점을 지나도록 만들어졌으므로
//   (updateBevGeometry 참고) 지면에서는 서로 평행하다.
//   즉 src 사다리꼴의 지면 대응 도형은 정확히 직사각형이고,
//   그 폭 W 와 길이 L 은
//
//     W/h = 2 * (bottom_width/2) * t(a_bot)/h / fx
//     L/h = D(a_top)/h - D(a_bot)/h
//
//   dst 사각형이 W 를 dst_w px, L 을 dst_h px 로 펴므로
//
//     cm_per_px_y = cm_per_px_x * (L/W) * (dst_w / dst_h)
//
// 반환하는 x_base_cm 은 BEV 하단 모서리의 지면 전방거리.
// camera_height_cm 이 0 이면 (미지정) 0 을 준다.
// ================================================================

bool KauLaneDetectionNode::bevScale(
    double & sx,
    double & sy,
    double & x_base_cm) const
{
    if (
        !camera_calibrated_ ||
        camera_matrix_.empty() ||
        bev_dst_points_.size() != 4 ||
        camera_height_cm_ <= 1e-6 ||
        lane_width_cm_ <= 1e-6
    )
    {
        return false;
    }


    const double fx = camera_matrix_.at<double>(0, 0);

    const double fy = camera_matrix_.at<double>(1, 1);

    const double cy = camera_matrix_.at<double>(1, 2);


    if (fx <= 1e-6 || fy <= 1e-6)
    {
        return false;
    }


    const double a_h =
        (bev_vanishing_y_ - cy) / fy;

    const double a_bot =
        (bev_src_bottom_y_ - cy) / fy;

    const double a_top =
        (bev_src_top_y_ - cy) / fy;


    // 두 행 모두 지평선 아래(지면 위)여야 한다
    if (
        a_bot - a_h < 1e-6 ||
        a_top - a_h < 1e-6
    )
    {
        return false;
    }


    const auto ground_d =
        [&](double a)
        {
            return (1.0 + a * a_h) / (a - a_h);
        };

    const auto ray_t =
        [&](double a)
        {
            return std::sqrt(1.0 + a_h * a_h) / (a - a_h);
        };


    // h = 1 로 둔 무차원 값 (비에서 약분된다)
    const double w_ground =
        bev_src_bottom_width_ * ray_t(a_bot) / fx;

    const double l_ground =
        ground_d(a_top) - ground_d(a_bot);


    if (w_ground <= 1e-9 || l_ground <= 1e-9)
    {
        return false;
    }


    const double dst_w =
        static_cast<double>(
            bev_dst_points_[1].x - bev_dst_points_[0].x);

    const double dst_h =
        static_cast<double>(
            bev_dst_points_[2].y - bev_dst_points_[1].y);


    if (dst_w <= 1e-6 || dst_h <= 1e-6)
    {
        return false;
    }


    // w_ground / l_ground 는 h = 1 로 둔 무차원 값이므로
    // 실 카메라 높이를 곱하면 cm 가 된다.
    //
    //   sx = h * w_ground / dst_w      하단 모서리 지면 폭 / dst 폭
    //   sy = h * l_ground / dst_h      사다리꼴 지면 깊이 / dst 높이

    sx = camera_height_cm_ * w_ground / dst_w;

    sy = camera_height_cm_ * l_ground / dst_h;


    x_base_cm = camera_height_cm_ * ground_d(a_bot);


    // ------------------------------------------------------------
    // 로그
    //
    // 축척은 camera_height_cm 하나에서 나오므로 역산할 게 없다.
    // 대신 lane_width_cm 이 몇 px 에 해당하는지를 찍는다.
    // 이 값이 실제 BEV 상의 노란선-흰선 간격과 다르면
    // camera_height_cm 또는 lane_width_cm 이 틀린 것이다.
    // (imageCallback 이 매 프레임 실측과 대조해 경고한다)
    // ------------------------------------------------------------

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "BEV scale: 가로 %.4f cm/px, 세로 %.4f cm/px "
        "(카메라 높이 %.2f cm + intrinsic 유도, 비등방 %.2f배) "
        "-> 범위 %.0f x %.0f cm",
        sx,
        sy,
        camera_height_cm_,
        sx / sy,
        sx * dst_w,
        sy * dst_h
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "BEV ground: 카메라앞 %.1f ~ %.1f cm "
        "(경로 X 원점 = %.1f cm). 차로폭 %.1f cm = %.1f BEV px",
        x_base_cm,
        camera_height_cm_ * ground_d(a_top),
        x_base_cm + path_x_offset_cm_,
        lane_width_cm_,
        lane_width_cm_ / sx
    );


    return true;
}




// ================================================================
// 유도 축척 갱신
//
// lane_width_px_ 는 설정값이 아니라 물리량에서 나온다.
//
//   lane_width_px = lane_width_cm / sx
//
// sx 는 카메라 높이와 intrinsic, BEV 사다리꼴에서 유도되므로
// CameraInfo 가 오거나 BEV 기하가 바뀌면 다시 계산해야 한다.
//
// 유도에 실패하면 (CameraInfo 미수신 등) 이전 값을 유지한다.
// imageCallback 이 CameraInfo 없이는 조기 반환하므로
// 자리표시 값이 실제 검출에 쓰이지는 않는다.
// ================================================================

void KauLaneDetectionNode::refreshDerivedScale()
{
    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    if (!bevScale(sx, sy, x_base))
    {
        return;
    }


    const double derived =
        lane_width_cm_ / sx;


    if (
        derived < 4.0 ||
        derived > 4.0 * static_cast<double>(bev_out_width_)
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "유도된 차로 폭 %.1f px 가 비정상입니다. "
            "camera_height_cm(%.2f) / lane_width_cm(%.1f) 확인. "
            "이전 값 %.1f px 유지.",
            derived,
            camera_height_cm_,
            lane_width_cm_,
            lane_width_px_
        );

        return;
    }


    if (std::abs(derived - lane_width_px_) > 0.05)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "차로 폭 유도: %.1f cm / %.4f cm/px = %.1f BEV px "
            "(이전 %.1f px)",
            lane_width_cm_,
            sx,
            derived,
            lane_width_px_
        );
    }


    lane_width_px_ = derived;
}


// ================================================================
// KauPath 발행 (docs/경로_형식.md 9.5)
//
// 절차:
//   1 제어점은 이미 quintic (buildCenterlinePath 에서 승격 완료)
//   2 호길이   Gauss-Legendre 10점  -> segLength
//   3 곡률상한 14차 근              -> kappaMaxExact
//   4 퇴화검사 isRegular            -> buildCenterlinePath 7절에서 완료
//   5 flat 적재
//
// segment 는 항상 1개다 (9.3, Lane Detection 의 표준 형태).
// 이산 좌표는 만들지도 싣지도 않는다. RViz 용 이산화만
// /viz/path/lane 으로 따로 나간다.
// ================================================================

void KauLaneDetectionNode::publishLanePath(
    const LanePath & path,
    const rclcpp::Time & stamp)
{
    if (
        !lane_path_publisher_ ||
        !path.valid
    )
    {
        return;
    }


    // 무결성: 수신측 검사(9.8)를 발행측에서 먼저 건다.
    if (
        path.ctrl.size() !=
        static_cast<std::size_t>(kau::bezier::NCTRL)
    )
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "제어점 개수가 %zu 개입니다 (기대 %d). 발행하지 않습니다.",
            path.ctrl.size(),
            kau::bezier::NCTRL
        );

        return;
    }


    kau_msgs::msg::KauPath msg;

    msg.header.stamp = stamp;

    msg.header.frame_id = path_frame_id_;

    msg.source = kau_msgs::msg::KauPath::SRC_LANE;

    msg.degree =
        static_cast<uint8_t>(kau::bezier::DEGREE);

    msg.is_closed = false;

    // 전역 호길이 기준점 없음 (localization 비의존)
    msg.s_offset = 0.0;


    // ------------------------------------------------------------
    // flat 적재. segment 1개이므로 그대로 6개.
    // ------------------------------------------------------------

    msg.ctrl_x.reserve(kau::bezier::NCTRL);

    msg.ctrl_y.reserve(kau::bezier::NCTRL);


    for (const kau::bezier::Point2 & cp : path.ctrl)
    {
        msg.ctrl_x.push_back(cp.x);

        msg.ctrl_y.push_back(cp.y);
    }


    msg.seg_length = { path.length_cm };

    msg.seg_kappa_max = {
        static_cast<float>(path.kappa_max)
    };

    msg.total_length = path.length_cm;

    msg.confidence =
        static_cast<float>(path.confidence);

    msg.valid_length = path.valid_length_cm;


    lane_path_publisher_->publish(msg);


    // ------------------------------------------------------------
    // RViz 전용 이산화 (9.9)
    //
    // 구독자가 없으면 만들지 않는다. 제어 경로는 이 값을
    // 쓰지 않으므로 대전제에 저촉되지 않는다.
    //
    // nav_msgs/Path 는 ROS 표준이므로 단위가 m 다.
    // KauPath 는 팀 규약대로 cm 이므로 여기서만 100 으로 나눈다.
    // ------------------------------------------------------------

    if (
        !viz_path_publisher_ ||
        viz_path_publisher_->get_subscription_count() == 0
    )
    {
        return;
    }


    constexpr int VIZ_SAMPLES = 40;

    nav_msgs::msg::Path viz;

    viz.header = msg.header;

    viz.poses.reserve(VIZ_SAMPLES);


    for (int i = 0; i < VIZ_SAMPLES; ++i)
    {
        const double u =
            static_cast<double>(i) /
            static_cast<double>(VIZ_SAMPLES - 1);

        const kau::bezier::Point2 p =
            kau::bezier::evalSeg(path.ctrl, u);

        const double th =
            kau::bezier::heading(path.ctrl, u);


        geometry_msgs::msg::PoseStamped ps;

        ps.header = msg.header;

        ps.pose.position.x = 0.01 * p.x;

        ps.pose.position.y = 0.01 * p.y;

        ps.pose.position.z = 0.0;

        ps.pose.orientation.z = std::sin(0.5 * th);

        ps.pose.orientation.w = std::cos(0.5 * th);


        viz.poses.push_back(ps);
    }


    viz_path_publisher_->publish(viz);
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

const char * KauLaneDetectionNode::gateName(int code)
{
    switch (code)
    {
        case 1:  return "근거부족";
        case 2:  return "조향한계";
        case 3:  return "시야이탈";
        default: return "ok";
    }
}


// cv::putText 는 Hershey 폰트라 CJK 가 물음표로 나온다.
const char * KauLaneDetectionNode::gateNameAscii(int code)
{
    switch (code)
    {
        case 1:  return "THIN";
        case 2:  return "STEER";
        case 3:  return "OUT";
        default: return "ok";
    }
}


// ================================================================
// 자차 종축 폴리라인
//
// src 사다리꼴의 옆변은 소실점 (bev_src_center_x_,
// bev_vanishing_y_) 을 지나도록 만들어져 있다. 요각 0 인
// 카메라에서 지면 평행선의 소실점 열은 주점 cx 이므로,
// bev_src_center_x_ == cx 일 때 사다리꼴의 대칭축은 지면에서
// 횡거리 0 인 직선, 즉 차량 종축과 같다.
//
// 아래(가까움) -> 위(멂) 순서라 진행방향과 같고, 이 폴리라인을
// 기준선으로 쓰면 lateralOffsets 의 부호가 그대로 "차량 오른쪽"
// 이 된다.
// ================================================================

std::vector<cv::Point2d> KauLaneDetectionNode::egoAxis(
    int bev_height) const
{
    double ex = 0.5 * static_cast<double>(bev_out_width_);


    if (bev_dst_points_.size() == 4)
    {
        ex =
            0.5 *
            static_cast<double>(
                bev_dst_points_[0].x + bev_dst_points_[1].x);
    }


    return {
        cv::Point2d(ex, static_cast<double>(bev_height - 1)),
        cv::Point2d(ex, 0.0)
    };
}




// ================================================================
// 관측선 -> 도로 단면의 자리
//
// 선이 하나만 보일 때 그것이 왼쪽 도로 경계인지 오른쪽
// 경계인지는 관측만으로는 알 수 없다. 히스토그램이 화면
// 반쪽 중 어디서 찾았는지는 근거가 못 된다. 코너에서 차선이
// 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
//
// 맵 단면(road_map.hpp)이 있으면 답이 나온다. 자차가 중앙선
// 에서 몇 cm 떨어져 있는지 알면 각 선이 자차 종축에서 몇 cm
// 떨어져 있어야 하는지가 정해진다.
//
//   자차가 중앙선 위에 있다고 두면 ego = 0cm
//
//     왼쪽 흰선 (-31.25)  기대 횡거리 -31.25cm
//     중앙선    (  0.00)  기대 횡거리   0.00cm
//     오른쪽흰선(+31.25)  기대 횡거리 +31.25cm
//
// 흰선 두 후보가 62.5cm 떨어져 있으므로 판정 여유가 넉넉하다.
// 실측이 어느 후보에서도 0.75W 밖이면 도로의 선이 아니다
// (연석, 체커보드, 다른 구간 차선). 그때는 고르지 않는다.
//
// 기준점 ego_offset_cm 은 centerOffsetPriorCm() 이 준다. 노란선
// 이나 흰선 두 개로 잰 값이 아직 싱싱하면 그것을 쓰고, 없으면
// 중앙선 위에 있다고 둔다.
//
// 흰선 하나로 얻은 값(src 3)은 절대 기준점이 되지 않는다. 그
// 값은 이 기준점으로 자리를 골라서 나온 것이라, 되먹이면 오차가
// 스스로를 키운다.
// ================================================================

double KauLaneDetectionNode::centerOffsetPriorCm() const
{
    // 30 프레임 = 14Hz 에서 약 2 초.
    if (center_prior_age_ <= 30)
    {
        return center_prior_offset_cm_;
    }


    // 근거가 없을 때의 최선의 가정. 차로 개념이 없으므로
    // "지정 차선 중심" 같은 기본값은 없다 — 중앙선 자체가
    // 목표이니 근거가 없을 때도 중앙선 위에 있다고 둔다.
    return 0.0;
}




int KauLaneDetectionNode::identifyLine(
    const std::vector<cv::Point2d> & track,
    kau_road::LineType type,
    double ego_offset_cm,
    double sx,
    int bev_height) const
{
    if (track.size() < 2 || sx <= 1e-9)
    {
        return -1;
    }


    const double d_cm =
        medianLateralOffset(
            egoAxis(bev_height),
            track
        ) * sx;


    const double ego_cm = ego_offset_cm;


    int best = -1;

    double best_res = 0.75 * kau_road::LANE_WIDTH_CM;


    for (
        std::size_t i = 0;
        i < kau_road::LINE_COUNT;
        ++i
    )
    {
        if (kau_road::LINES[i].type != type)
        {
            continue;
        }


        const double res =
            std::abs(
                d_cm -
                (kau_road::LINES[i].offset_cm - ego_cm));


        if (res < best_res)
        {
            best_res = res;

            best = static_cast<int>(i);
        }
    }


    return best;
}




// ================================================================
// 중앙선 대비 자차 위치 갱신
//
// 원리는 한 줄이다. 중앙선을 기준선으로 두고 자차 위치의 부호
// 있는 횡거리를 재면, 그게 곧 자차가 중앙선을 얼마나 잘 따라가고
// 있는지다.
//
// 차로 개념이 없으므로 이 값으로 "차로 번호"를 매기거나
// "지정 차선 이탈"을 판정하지 않는다. 순수하게 진단값이고,
// identifyLine 이 흰선 하나를 disambiguate 할 때 쓰는 기준점이다.
//
// 세 가지를 조심해야 한다.
//
// 1. 기준선은 반드시 "관측된" 선이어야 한다.
//    16-c 소실 복원은 남은 선에서 차로 폭만큼 평행이동해
//    없는 선을 만들어 낸다. 그 선으로 이 값을 재면
//    "차로 폭만큼 옆에 있을 것" 이라는 가정을 관측처럼
//    되돌려 읽는 순환이 된다. 그래서 16-b 에서, 복원 전에
//    부른다.
//
// 2. 자차 위치는 BEV 안에 없다. BEV 아랫변은 카메라 앞
//    x_base_cm(약 32cm) 지점이고 차량은 그보다 뒤에 있다.
//    관측 가능한 가장 가까운 점(BEV 맨 아랫줄의 중앙 열)을
//    자차 대신 쓴다. 횡방향으로는 같은 직선 위이므로 영향이
//    없고, 요각 오차만큼만 앞선 값이 된다.
//
// 3. 근거(노란선 또는 흰선 두 개)가 끊겨도 몇 프레임은 직전
//    값을 들고 간다. 점선 공백과 순간 가림 대응.
// ================================================================

KauLaneDetectionNode::CenterFix KauLaneDetectionNode::resolveCenterOffset(
    const LaneDetectionResult & left,
    const LaneDetectionResult & yellow,
    const LaneDetectionResult & right,
    int bev_height)
{
    // prior 나이 먹이기. 아래에서 확실한 근거가 나오면 0 으로
    // 되돌아간다. 포화시켜 두면 오래 굶어도 넘치지 않는다.
    if (center_prior_age_ < 1000000)
    {
        ++center_prior_age_;
    }


    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    const bool have_scale =
        bevScale(sx, sy, x_base) &&
        lane_width_px_ > 1e-6 &&
        lane_width_cm_ > 1e-6 &&
        bev_dst_points_.size() == 4;


    // ------------------------------------------------------------
    // 자차 종축 = BEV 중앙 열 (egoAxis 참고)
    // ------------------------------------------------------------

    if (have_scale && !camera_matrix_.empty())
    {
        const double cx = camera_matrix_.at<double>(0, 2);

        if (std::abs(bev_src_center_x_ - cx) > 2.0)
        {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "bev_src_center_x(%.1f) 가 주점 cx(%.1f) 와 %.1fpx "
                "다릅니다. BEV 중앙 열이 차량 종축과 어긋나므로 "
                "중앙선 대비 위치 판정에 그만큼 편향이 실립니다.",
                bev_src_center_x_,
                cx,
                std::abs(bev_src_center_x_ - cx)
            );
        }
    }


    const auto observed =
        [](const LaneDetectionResult & l)
        {
            return l.valid &&
                   l.found_count > 0 &&
                   l.track_px.size() >= 2;
        };


    // ------------------------------------------------------------
    // 기준선 = 중앙선
    // ------------------------------------------------------------

    std::vector<cv::Point2d> center_ref;

    int src = 0;


    if (have_scale && observed(yellow))
    {
        center_ref = yellow.track_px;

        src = 1;
    }
    else if (have_scale && observed(left) && observed(right))
    {
        // 노란선이 점선 공백에 걸려 안 보여도, 흰선 두 개가
        // 도로 양 가장자리라면 중앙선은 그 한가운데다.
        //
        // 간격이 도로 폭(2W)에 맞을 때만 인정한다. 좌/우가 같은
        // 선을 물었거나 배경 차선을 물었으면 간격이 어긋난다.
        const double gap =
            medianLateralOffset(
                left.track_px,
                right.track_px
            );

        const double want = 2.0 * lane_width_px_;


        if (
            std::abs(gap) > 0.75 * want &&
            std::abs(gap) < 1.25 * want
        )
        {
            center_ref =
                offsetTrack(
                    left.track_px,
                    0.5 * gap
                );

            src = 2;
        }
    }
    else if (
        have_scale &&
        (observed(left) != observed(right))
    )
    {
        // ------------------------------------------------------
        // 흰선이 하나뿐
        //
        // 이 경우가 코너에서 제일 흔하다. 안쪽 차선이 BEV 옆으로
        // 먼저 빠져나가고 바깥 차선만 남는다.
        //
        // 관측만으로는 남은 그 선이 도로의 왼쪽 가장자리인지
        // 오른쪽 가장자리인지 알 수 없다. 맵 단면이 필요하다.
        // identifyLine 이 자차 종축 대비 실측 횡거리를 단면의
        // 기대 위치와 맞춰 자리를 고른다.
        //
        // 자리가 정해지면 그만큼 되밀어 중앙선을 세운다.
        // ------------------------------------------------------

        const LaneDetectionResult & one =
            observed(left) ? left : right;

        const int k =
            identifyLine(
                one.track_px,
                kau_road::LineType::White,
                centerOffsetPriorCm(),
                sx,
                bev_height
            );


        if (
            k >= 0 &&
            std::abs(kau_road::LINES[k].offset_cm) > 1e-9
        )
        {
            center_ref =
                offsetTrack(
                    one.track_px,
                    -kau_road::LINES[k].offset_cm / sx
                );

            src = 3;
        }
    }


    // ------------------------------------------------------------
    // 근거가 없으면 직전 판정을 hold_frames 까지 끌고 간다
    // ------------------------------------------------------------

    if (center_ref.size() < 2)
    {
        if (
            center_fix_.valid &&
            center_fix_miss_ < center_hold_frames_
        )
        {
            ++center_fix_miss_;

            center_fix_.source = 4;

            return center_fix_;
        }


        center_fix_ = CenterFix();

        return center_fix_;
    }


    // ------------------------------------------------------------
    // 부호 있는 횡거리
    // ------------------------------------------------------------

    const cv::Point2d ego_px =
        egoAxis(bev_height).front();

    const double d_cm =
        lateralOffsetAt(center_ref, ego_px) * sx;


    // 도로 폭 밖이면 기준선이 중앙선이 아니다.
    const double road_half_cm =
        kau_road::roadHalfWidthCm();


    if (std::abs(d_cm) > road_half_cm + 0.5 * lane_width_cm_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "자차가 중앙선에서 %+.1fcm 로 나옵니다. 도로 반폭 "
            "%.1fcm 를 넘으므로 기준선이 중앙선이 아닙니다. "
            "판정을 보류합니다.",
            d_cm,
            road_half_cm
        );


        if (
            center_fix_.valid &&
            center_fix_miss_ < center_hold_frames_
        )
        {
            ++center_fix_miss_;

            center_fix_.source = 4;

            return center_fix_;
        }


        center_fix_ = CenterFix();

        return center_fix_;
    }


    center_fix_miss_ = 0;


    // ------------------------------------------------------------
    // 결과
    // ------------------------------------------------------------

    CenterFix fix;

    fix.valid = true;

    fix.offset_cm = d_cm;

    fix.source = src;


    // 확실한 근거로 잰 값만 다음 프레임의 기준점이 된다.
    if (src == 1 || src == 2)
    {
        center_prior_offset_cm_ = d_cm;

        center_prior_age_ = 0;
    }


    center_fix_ = fix;

    return center_fix_;
}




int KauLaneDetectionNode::pathGate(
    const LanePath & path,
    double sx) const
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

    const double steer_rad =
        max_steer_deg_ * M_PI / 180.0;

    if (
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
    // BEV 영상 밖으로 나가는 경로는 관측 근거가 없다.
    // ------------------------------------------------------------

    double limit = max_lateral_cm_;

    if (limit <= 0.0)
    {
        limit = 0.5 * bev_out_width_ * sx;
    }


    for (const kau::bezier::Point2 & c : path.ctrl)
    {
        if (std::abs(c.y - path_y_offset_cm_) > limit)
        {
            return 3;
        }
    }


    return 0;
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

KauLaneDetectionNode::LanePath KauLaneDetectionNode::buildCenterlinePath(
    const LaneDetectionResult & left,
    const LaneDetectionResult & yellow,
    const LaneDetectionResult & right,
    int bev_width,
    int bev_height,
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
    };


    const Source sources[3] =
    {
        { &left,    lane_width_px_ },
        { &yellow,  0.0            },
        { &right,  -lane_width_px_ }
    };


    struct Candidate
    {
        std::vector<cv::Point2d> pts;

        double weight;
    };


    std::vector<Candidate> cands;


    for (const Source & src : sources)
    {
        if (
            !src.lane->valid ||
            src.lane->found_count <= 0 ||
            src.lane->track_px.size() < 2
        )
        {
            continue;
        }


        std::vector<cv::Point2d> c =
            offsetTrack(
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
            projectOnPolyline(
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
        !polyFitW(ss, xs, ws, order, cx_s) ||
        !polyFitW(ss, ys, ws, order, cy_s)
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
                polyEval(cx_s, s),
                polyEval(cy_s, s)
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

    path.reject = pathGate(path, sx);

    if (path_gate_enable_ && path.reject != 0)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "경로 기각(%s): 창 %d개, 반경 %.0fcm",
            gateName(path.reject),
            path.source_windows,
            (path.kappa_max > 1e-9) ? 1.0 / path.kappa_max : 0.0
        );

        return path;
    }


    path.valid = true;


    return path;
}




// ================================================================
// 발행한 Bezier 를 BEV 위에 되돌려 그린다.
//
// sampling 은 시각화 전용 (대전제: 연산에 이산 좌표 금지).
// ================================================================

void KauLaneDetectionNode::drawPathOverlay(
    const LanePath & path,
    cv::Mat & image)
{
    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    if (!path.built || !bevScale(sx, sy, x_base))
    {
        return;
    }


    // 통과 = 마젠타, 기각 = 붉은 회색.
    // 기각된 경로도 그려야 무엇이 막혔는지 눈으로 보인다.
    const cv::Scalar curve_color =
        path.valid
            ? cv::Scalar(255, 0, 255)
            : cv::Scalar(60, 60, 200);


    const double h_px = static_cast<double>(image.rows);

    const double cx_bev = 0.5 * image.cols;


    const auto to_px =
        [&](const kau::bezier::Point2 & p)
        {
            return cv::Point(
                static_cast<int>(std::lround(
                    cx_bev - (p.y - path_y_offset_cm_) / sx)),
                static_cast<int>(std::lround(
                    h_px -
                    (p.x - x_base - path_x_offset_cm_) / sy))
            );
        };


    // 제어 다각형
    for (
        std::size_t i = 1;
        i < path.ctrl.size();
        ++i
    )
    {
        cv::line(
            image,
            to_px(path.ctrl[i - 1]),
            to_px(path.ctrl[i]),
            cv::Scalar(120, 120, 120),
            1
        );
    }


    // 곡선
    const std::vector<kau::bezier::Point2> pts =
        kau::bezier::sample(path.ctrl, 60);


    for (
        std::size_t i = 1;
        i < pts.size();
        ++i
    )
    {
        cv::line(
            image,
            to_px(pts[i - 1]),
            to_px(pts[i]),
            cv::Scalar(255, 0, 255),
            2
        );
    }


    // 제어점
    for (const kau::bezier::Point2 & p : path.ctrl)
    {
        cv::circle(
            image,
            to_px(p),
            4,
            curve_color,
            -1
        );
    }
}




// ================================================================
// Image Callback
// ================================================================

void KauLaneDetectionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // ========================================================
        // 1. ROS Image -> OpenCV
        // ========================================================

        cv::Mat frame =
            cv_bridge::toCvCopy(
                msg,
                sensor_msgs::image_encodings::BGR8
            )->image;


        if (frame.empty())
        {
            return;
        }


        // ========================================================
        // CameraInfo 대기
        // ========================================================

        if (!camera_calibrated_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for CameraInfo..."
            );

            return;
        }


        // ========================================================
        // 2. Camera Undistortion
        // ========================================================

        cv::Mat undistorted_frame;


        cv::undistort(
            frame,
            undistorted_frame,
            camera_matrix_,
            distortion_coefficients_
        );


        // --------------------------------------------------------
        // Publish Undistorted Image
        // --------------------------------------------------------

        auto undistorted_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                undistorted_frame
            ).toImageMsg();


        undistorted_image_publisher_->publish(
            *undistorted_msg
        );


        // ========================================================
        // 3. BEV
        //
        // src 사다리꼴 / dst 사각형은 파라미터로 관리
        // ========================================================

        refreshBevParameters();


        // --------------------------------------------------------
        // src ROI Overlay
        //
        // 원본 위에 사다리꼴을 그려서
        // 실제 차선과 좌/우 변이 나란한지 눈으로 확인한다.
        // --------------------------------------------------------

        if (publish_roi_overlay_)
        {
            cv::Mat roi_overlay =
                undistorted_frame.clone();


            std::vector<cv::Point> roi_polygon;


            for (const auto & point : bev_src_points_)
            {
                roi_polygon.push_back(
                    cv::Point(
                        static_cast<int>(point.x),
                        static_cast<int>(point.y)
                    )
                );
            }


            cv::polylines(
                roi_overlay,
                roi_polygon,
                true,
                cv::Scalar(
                    0,
                    255,
                    0
                ),
                2
            );


            auto roi_msg =
                cv_bridge::CvImage(
                    msg->header,
                    sensor_msgs::image_encodings::BGR8,
                    roi_overlay
                ).toImageMsg();


            roi_overlay_publisher_->publish(
                *roi_msg
            );
        }


        cv::Mat bev_frame;


        // INTER_CUBIC.
        //
        // src 사다리꼴은 원본의 아주 작은 조각이다.
        //   위쪽 변 84.4px -> dst 240px  (2.84배 확대)
        //   세로   75px    -> dst 360px  (4.80배 확대)
        // 즉 원거리 1픽셀이 BEV 에서 13.6픽셀 면적이 된다.
        //
        // 없는 정보가 생기지는 않지만, 보간이 부드러워지면
        // 확대된 마킹 가장자리의 색이 덜 뭉개져 HLS 임계를
        // 통과하는 픽셀이 늘어난다.
        //
        // 실패 77장 실측:
        //   LINEAR : 노랑 중앙 880px  노란선 추적 스텝합 420
        //   CUBIC  : 노랑 중앙1045px  노란선 추적 스텝합 469
        cv::warpPerspective(
            undistorted_frame,
            bev_frame,
            perspective_matrix_,
            cv::Size(
                bev_out_width_,
                bev_out_height_
            ),
            cv::INTER_CUBIC
        );


        // --------------------------------------------------------
        // Publish BEV
        // --------------------------------------------------------

        auto bev_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                bev_frame
            ).toImageMsg();


        bev_image_publisher_->publish(
            *bev_msg
        );


        // ========================================================
        // 4. HLS 변환
        // ========================================================

        cv::Mat hls;


        cv::cvtColor(
            bev_frame,
            hls,
            cv::COLOR_BGR2HLS
        );


        // ========================================================
        // 5. Yellow Mask
        // ========================================================

        cv::Mat yellow_mask;


        cv::inRange(
            hls,
            toScalar(yellow_hls_lo_),
            toScalar(yellow_hls_hi_),
            yellow_mask
        );


        // ========================================================
        // 6. White Mask
        // ========================================================

        cv::Mat white_mask;


        cv::inRange(
            hls,
            toScalar(white_hls_lo_),
            toScalar(white_hls_hi_),
            white_mask
        );


        // ========================================================
        // 7. Combined HLS Binary
        //
        // Sobel 제거
        // Edge 제거
        //
        // Yellow + White만 사용
        // ========================================================

        cv::Mat combined_binary;


        cv::bitwise_or(
            yellow_mask,
            white_mask,
            combined_binary
        );


        // ========================================================
        // 8. 작은 Noise 제거
        // ========================================================

        cv::Mat kernel =
            cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(
                    3,
                    3
                )
            );


        // --------------------------------------------------------
        // 예전에는 combined_binary 에만 걸었는데,
        // 슬라이딩 윈도우가 쓰는 건 yellow_mask / white_mask 라서
        // 정리 효과가 탐지에 전혀 반영되지 않았다.
        //
        // OPEN  : 점 노이즈 제거
        // CLOSE : 점선 차선의 작은 끊김 메우기 (곡선 끊김 완화)
        // dilate: 살짝 두껍게 -> 창별 픽셀 수 안정
        // --------------------------------------------------------

        for (
            cv::Mat * mask_ptr :
            { &yellow_mask, &white_mask }
        )
        {
            cv::morphologyEx(
                *mask_ptr,
                *mask_ptr,
                cv::MORPH_OPEN,
                kernel
            );

            cv::morphologyEx(
                *mask_ptr,
                *mask_ptr,
                cv::MORPH_CLOSE,
                kernel
            );

            cv::dilate(
                *mask_ptr,
                *mask_ptr,
                kernel
            );
        }


        cv::bitwise_or(
            yellow_mask,
            white_mask,
            combined_binary
        );


        // ========================================================
        // 9. HLS Binary Publish
        // ========================================================

        auto hls_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::MONO8,
                combined_binary
            ).toImageMsg();


        hls_binary_publisher_->publish(
            *hls_msg
        );


        // ========================================================
        // 10. Combined Binary Publish
        // ========================================================

        auto combined_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::MONO8,
                combined_binary
            ).toImageMsg();


        combined_binary_publisher_->publish(
            *combined_msg
        );


        // ========================================================
        // 11. Debug Image
        // ========================================================

        cv::Mat debug_image =
            bev_frame.clone();


        // ========================================================
        // 12. Histogram Base Detection
        //
        // 노란 중앙선을 앵커로 삼는다.
        //
        // 흰선을 화면 반쪽 전체에서 찾으면, 곡선에서 한쪽
        // 흰선이 프레임 밖으로 나갔을 때 멀리 있는 노이즈를
        // 차선으로 잡는다. 노랑 기준 [0.4W ~ 1.6W] 밴드
        // 안에서만 찾는다.
        // ========================================================

        // 좁은 밴드가 점선 공백에 걸려 넓힌 경우 band > 기본값이 된다.
        // 그때의 base 는 곡선에서 부정확하므로 상태로 내보낸다.
        double yellow_band = 0.0;

        const int yellow_base =
            findHistogramPeak(
                yellow_mask,
                0,
                yellow_mask.cols,
                &yellow_band
            );


        int left_base = -1;

        int right_base = -1;


        if (yellow_base >= 0)
        {
            left_base =
                findHistogramPeak(
                    white_mask,
                    static_cast<int>(
                        yellow_base - 1.6 * lane_width_px_
                    ),
                    static_cast<int>(
                        yellow_base - 0.4 * lane_width_px_
                    )
                );

            right_base =
                findHistogramPeak(
                    white_mask,
                    static_cast<int>(
                        yellow_base + 0.4 * lane_width_px_
                    ),
                    static_cast<int>(
                        yellow_base + 1.6 * lane_width_px_
                    )
                );
        }
        else
        {
            // 노랑 소실 -> 반쪽 전체 탐색으로 폴백
            left_base =
                findHistogramPeak(
                    white_mask,
                    0,
                    white_mask.cols / 2
                );

            right_base =
                findHistogramPeak(
                    white_mask,
                    white_mask.cols / 2,
                    white_mask.cols
                );
        }


        // ========================================================
        // 13. Lane Detection Result
        // ========================================================

        LaneDetectionResult left_lane;

        LaneDetectionResult yellow_lane;

        LaneDetectionResult right_lane;


        // ========================================================
        // 14. Left White Sliding Window
        // ========================================================

        if (left_base >= 0)
        {
            left_lane =
                detectLaneSlidingWindow(
                    white_mask,
                    left_base,
                    debug_image,
                    cv::Scalar(
                        255,
                        0,
                        0
                    )
                );
        }


        // ========================================================
        // 15. Center Yellow Sliding Window
        // ========================================================

        if (yellow_base >= 0)
        {
            yellow_lane =
                detectLaneSlidingWindow(
                    yellow_mask,
                    yellow_base,
                    debug_image,
                    cv::Scalar(
                        0,
                        255,
                        255
                    )
                );
        }


        // ========================================================
        // 16. Right White Sliding Window
        // ========================================================

        if (right_base >= 0)
        {
            right_lane =
                detectLaneSlidingWindow(
                    white_mask,
                    right_base,
                    debug_image,
                    cv::Scalar(
                        0,
                        0,
                        255
                    )
                );
        }


        // ========================================================
        // 16-a. 흰선 쪽 검증
        //
        // 흰선 base 는 x 구간 히스토그램으로 찾는다. 차선이
        // 세로로 서 있을 때만 "x 로 왼쪽" = "차로의 왼쪽" 이다.
        // 90도 코너에서 차선이 가로로 누우면 좌/우 흰선이
        // 노란선의 위/아래에 쌓이므로, x 구간이 반대쪽 선이나
        // 같은 선을 물어 온다.
        //
        // 추적이 끝난 뒤 노란선 기준으로 어느 쪽에 있는지 재보고
        // 반대쪽이면 버린다. 버린 자리는 아래 16-c 복원이 채운다.
        // ========================================================

        if (
            yellow_lane.valid &&
            yellow_lane.track_px.size() >= 2
        )
        {
            const double side_min =
                lane_side_min_ratio_ * lane_width_px_;


            // 검증을 통과한 흰선의 실측 간격.
            // 유도된 lane_width_px_ 와 대조해 축척을 자가 점검한다.
            std::vector<double> measured_width_px;


            const auto usable =
                [](const LaneDetectionResult & l)
                {
                    return l.valid &&
                           l.found_count > 0 &&
                           l.track_px.size() >= 2;
                };


            // ----------------------------------------------------
            // (1) 부호로 자리 재배정
            //
            // x 구간 히스토그램이 좌우를 바꿔 무는 일이 있다.
            // 예전에는 그냥 버렸는데, 그건 멀쩡한 검출을 이름만
            // 잘못 붙인 것이라 버릴 이유가 없다. 노란선 기준
            // 부호가 가리키는 자리로 옮긴다.
            //
            // 주행 로그 18.5분 실측 (경고 16건의 내역):
            //   완전 뒤바뀜 4건(25%)  <- 여기서 회수
            //   같은 선     5건(31%)  <- 아래 (2) 에서 정리
            //   중간 표류   7건(44%)  <- 아래 (3) 회랑에서 절단
            // ----------------------------------------------------

            {
                struct Seen
                {
                    LaneDetectionResult lane;

                    double off;
                };


                std::vector<Seen> seen;


                for (LaneDetectionResult * l : { &left_lane, &right_lane })
                {
                    if (usable(*l))
                    {
                        seen.push_back(
                            Seen{
                                *l,
                                medianLateralOffset(
                                    yellow_lane.track_px,
                                    l->track_px
                                )
                            }
                        );
                    }
                }


                left_lane = LaneDetectionResult();

                right_lane = LaneDetectionResult();


                // 같은 쪽을 문 것끼리는 근거가 많은 하나만 남는다
                for (const Seen & s : seen)
                {
                    LaneDetectionResult * slot =
                        (s.off <= -side_min) ? &left_lane :
                        (s.off >=  side_min) ? &right_lane : nullptr;


                    if (slot == nullptr)
                    {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "흰선이 노란선과 겹칩니다 "
                            "(대비 %+.0fpx, 기대 %s%.0fpx). 버립니다.",
                            s.off,
                            "+-",
                            side_min
                        );

                        continue;
                    }


                    if (slot->found_count >= s.lane.found_count &&
                        slot->track_px.size() >= 2)
                    {
                        continue;
                    }


                    *slot = s.lane;
                }
            }


            // ----------------------------------------------------
            // (3) 회랑 이탈 지점에서 절단
            //
            // 차로 폭은 고정이다. 노란선에서 그만큼 떨어져 있지
            // 않은 점은 우리 차로의 흰선이 아니라 체커보드 연석
            // 이거나 급커브에서 보이는 다른 구간 차선이다.
            //
            // 통째로 버리지 않고 갈아탄 지점에서 자른다.
            // ----------------------------------------------------

            const double lo_px = lane_corridor_lo_ * lane_width_px_;

            const double hi_px = lane_corridor_hi_ * lane_width_px_;


            const auto clip_side =
                [&](
                    LaneDetectionResult & lane,
                    double want_sign,
                    const char * name)
                {
                    if (!usable(lane))
                    {
                        return;
                    }


                    const std::size_t before =
                        lane.track_px.size();


                    lane.track_px =
                        truncateAtCorridor(
                            lane.track_px,
                            yellow_lane.track_px,
                            want_sign,
                            lo_px,
                            hi_px
                        );


                    if (lane.track_px.size() == before)
                    {
                        measured_width_px.push_back(
                            std::abs(
                                medianLateralOffset(
                                    yellow_lane.track_px,
                                    lane.track_px
                                )
                            )
                        );

                        return;
                    }


                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "%s 흰선이 %zu번째 스텝에서 차로를 벗어났습니다 "
                        "(회랑 %.0f~%.0fpx). 그 지점에서 자릅니다.",
                        name,
                        lane.track_px.size(),
                        lo_px,
                        hi_px
                    );


                    lane.found_count =
                        static_cast<int>(lane.track_px.size());


                    if (lane.track_px.size() < MIN_VALID_WINDOWS)
                    {
                        lane = LaneDetectionResult();
                    }
                };


            clip_side(left_lane,  -1.0, "왼쪽");

            clip_side(right_lane, +1.0, "오른쪽");


            // ----------------------------------------------------
            // 축척 자가 점검
            //
            // lane_width_px_ 는 camera_height_cm 과 intrinsic 에서
            // 유도한 값이다. 실측 간격이 여기서 크게 벗어나면
            // 카메라 높이나 차로 폭 설정이 틀린 것이고, 발행되는
            // 모든 좌표가 같은 비율로 틀어진다.
            //
            // 곡선 구간에서는 재지 않는다.
            //
            // 실측은 흰선의 각 점을 노란선 "폴리라인" 에 투영해
            // 잰다. 폴리라인은 호를 현으로 근사한 것이라 곡선
            // 바깥쪽에서는 실제 호까지의 거리보다 멀게 나온다.
            // 스텝 25px 짜리 현이면 곡률반경 100cm 구간에서
            // 몇 % 가 부풀려지고, 그것이 lane_width_cm 을 35.0
            // 으로 잘못 잡게 만든 원인이었다 (실측 중앙값 108.3
            // px, 직선에서 다시 재면 97.5 px).
            //
            // 현/호 비가 0.98 이상, 즉 거의 직선일 때만 신뢰한다.
            // ----------------------------------------------------

            double yellow_straightness = 0.0;

            {
                double arc = 0.0;

                for (
                    std::size_t i = 1;
                    i < yellow_lane.track_px.size();
                    ++i
                )
                {
                    arc +=
                        cv::norm(
                            yellow_lane.track_px[i] -
                            yellow_lane.track_px[i - 1]);
                }


                if (arc > 1e-6)
                {
                    yellow_straightness =
                        cv::norm(
                            yellow_lane.track_px.back() -
                            yellow_lane.track_px.front()) / arc;
                }
            }


            // 점이 2개뿐이면 현/호 비가 무조건 1.0 이라 직선
            // 판정이 공짜로 통과한다. 점 수 하한을 같이 건다.
            if (
                !measured_width_px.empty() &&
                lane_width_px_ > 1e-6 &&
                yellow_lane.track_px.size() >= 6 &&
                yellow_straightness > 0.98
            )
            {
                std::sort(
                    measured_width_px.begin(),
                    measured_width_px.end()
                );

                const double measured =
                    measured_width_px[measured_width_px.size() / 2];

                const double ratio =
                    measured / lane_width_px_;


                if (ratio < 0.85 || ratio > 1.15)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        10000,
                        "축척 불일치: 차로 폭 실측 %.1f px vs "
                        "유도 %.1f px (%.0f%%). "
                        "camera_height_cm(%.2f) 또는 "
                        "lane_width_cm(%.1f) 을 확인하십시오. "
                        "발행 좌표가 같은 비율로 틀어집니다.",
                        measured,
                        lane_width_px_,
                        100.0 * ratio,
                        camera_height_cm_,
                        lane_width_cm_
                    );
                }
            }
        }
        else if (
            left_lane.valid &&
            right_lane.valid &&
            left_lane.found_count > 0 &&
            right_lane.found_count > 0 &&
            left_lane.track_px.size() >= 2 &&
            right_lane.track_px.size() >= 2
        )
        {
            // 노란선이 없으면 기준이 없다. 이때 흰선 base 는
            // 화면 반쪽 탐색으로 떨어지는데, 코너에서는 좌/우가
            // 같은 흰선을 무는 일이 잦다. 서로 겹치면 근거가
            // 많은 쪽만 남긴다.
            const double gap =
                std::abs(
                    medianLateralOffset(
                        left_lane.track_px,
                        right_lane.track_px
                    )
                );


            if (gap < 0.5 * lane_width_px_)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "좌/우 흰선이 같은 선을 물었습니다 "
                    "(간격 %.0fpx). 근거가 적은 쪽을 버립니다.",
                    gap
                );


                if (left_lane.found_count >= right_lane.found_count)
                {
                    right_lane = LaneDetectionResult();
                }
                else
                {
                    left_lane = LaneDetectionResult();
                }
            }
        }
        else
        {
            // ----------------------------------------------------
            // 노란선도 없고 흰선도 하나뿐
            //
            // 여기가 코너에서 제일 흔한 상태다. 안쪽 차선이 BEV
            // 옆으로 먼저 빠져나가고 바깥 차선만 남는다.
            //
            // 이때 그 선이 어느 쪽 선인지를 히스토그램이 화면
            // 반쪽 중 어디서 찾았는지로 정하고 있었다. 차선이
            // 세로로 서 있을 때만 맞는 규칙이다. 90도 코너에서
            // 차선이 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
            // 그러면 아래 16-c 복원이 반대 방향으로 차로 폭만큼
            // 밀어서, 있지도 않은 자리에 중심선을 세운다.
            //
            // 맵 단면(road_map.hpp)으로 자리를 고른다.
            // ----------------------------------------------------

            const auto observed_one =
                [](const LaneDetectionResult & l)
                {
                    return l.valid &&
                           l.found_count > 0 &&
                           l.track_px.size() >= 2;
                };

            const bool have_left  = observed_one(left_lane);

            const bool have_right = observed_one(right_lane);


            double sx_one = 0.0;

            double sy_one = 0.0;

            double xb_one = 0.0;


            if (
                (have_left != have_right) &&
                bevScale(sx_one, sy_one, xb_one)
            )
            {
                const LaneDetectionResult one =
                    have_left ? left_lane : right_lane;

                const int k =
                    identifyLine(
                        one.track_px,
                        kau_road::LineType::White,
                        centerOffsetPriorCm(),
                        sx_one,
                        bev_frame.rows
                    );


                if (k < 0)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "흰선 하나를 잡았는데 맵 단면의 어느 선에도 "
                        "맞지 않습니다 (자차 종축 대비 %+.1fcm). "
                        "도로 밖의 선으로 보고 버립니다.",
                        medianLateralOffset(
                            egoAxis(bev_frame.rows),
                            one.track_px) * sx_one
                    );

                    left_lane = LaneDetectionResult();

                    right_lane = LaneDetectionResult();
                }
                else
                {
                    const bool want_left =
                        kau_road::LINES[k].offset_cm < 0.0;


                    if (want_left != have_left)
                    {
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "흰선 하나를 %s 선으로 잡았는데 맵 단면상 "
                            "%s 입니다. 자리를 옮깁니다.",
                            have_left ? "왼쪽" : "오른쪽",
                            kau_road::LINES[k].name
                        );


                        left_lane = LaneDetectionResult();

                        right_lane = LaneDetectionResult();


                        if (want_left)
                        {
                            left_lane = one;
                        }
                        else
                        {
                            right_lane = one;
                        }
                    }
                }
            }
        }


        // ========================================================
        // 16-b. 중앙선 대비 자차 위치 갱신
        //
        // 반드시 아래 16-c 복원 앞이다. 복원된 선은 관측이
        // 아니라 "차로 폭만큼 옆에 있을 것" 이라는 가정이라,
        // 그것으로 이 값을 재면 가정을 관측처럼 되돌려 읽는
        // 순환이 된다.
        // ========================================================

        const CenterFix center_fix =
            resolveCenterOffset(
                left_lane,
                yellow_lane,
                right_lane,
                bev_frame.rows
            );


        // ========================================================
        // 16-c. 한쪽 소실 복원
        //
        // 곡선에서는 안쪽 차선이 BEV 프레임 옆으로 먼저
        // 빠져나가고 바깥 차선만 남는다. 이때 남은 차선에서
        // lane_width_px 만큼 평행이동해 사라진 쪽을 복원한다.
        //
        // 노란 중앙선이 기준이라 좌/우가 뒤바뀔 여지가 없어
        // 부호만 맞추면 끝이다.
        //
        // 노란선이 없고 흰선도 하나뿐인 경우가 문제였다. 그
        // 선이 어느 쪽 선인지 틀리면 여기서 차로 폭만큼 반대로
        // 밀어 완전히 다른 중심선을 만들어 낸다. 16-a 마지막
        // 분기가 맵 단면(road_map.hpp)으로 자리를 먼저 확정하고
        // 내려오므로, 여기서는 부호만 맞추면 된다.
        //
        // 오프셋은 맵 단면의 인접 선 간격 그대로다.
        //   흰선 <-> 중앙선  1W
        //   흰선 <-> 반대 흰선 2W
        // ========================================================

        const auto restore_from =
            [&](
                const LaneDetectionResult & source,
                double offset)
            {
                LaneDetectionResult restored;

                restored.detected = true;

                restored.valid = true;

                // 복원된 선은 실제로 본 것이 아니므로
                // found_count 는 물려주지 않는다.
                restored.found_count = 0;

                // 평행이동은 반드시 법선 방향.
                // x 방향 offset 은 차선이 세로일 때만 맞다.
                restored.track_px =
                    offsetTrack(
                        source.track_px,
                        offset
                    );


                return restored;
            };


        if (yellow_lane.valid)
        {
            if (!left_lane.valid)
            {
                left_lane =
                    restore_from(
                        yellow_lane,
                        -lane_width_px_
                    );
            }

            if (!right_lane.valid)
            {
                right_lane =
                    restore_from(
                        yellow_lane,
                        lane_width_px_
                    );
            }
        }
        else if (left_lane.valid && !right_lane.valid)
        {
            yellow_lane =
                restore_from(
                    left_lane,
                    lane_width_px_
                );

            right_lane =
                restore_from(
                    left_lane,
                    2.0 * lane_width_px_
                );
        }
        else if (right_lane.valid && !left_lane.valid)
        {
            yellow_lane =
                restore_from(
                    right_lane,
                    -lane_width_px_
                );

            left_lane =
                restore_from(
                    right_lane,
                    -2.0 * lane_width_px_
                );
        }


        // ========================================================
        // 16-d. 중심선 -> quintic Bezier 제어점 (공통 경로 형식)
        //
        // 이 노드의 최종 산출물. 이산 좌표가 아니라 제어점 6개다.
        // 차로를 묻지 않고 중앙선(황색 점선)만 추종한다.
        // ========================================================

        std::vector<cv::Point2d> center_pts_px;

        const LanePath lane_path =
            buildCenterlinePath(
                left_lane,
                yellow_lane,
                right_lane,
                bev_frame.cols,
                bev_frame.rows,
                center_pts_px
            );


        // ========================================================
        // 16-e. 경로 발행
        //
        // 게이트를 통과한 경로만 나간다. 기각된 경로는 발행하지
        // 않고 debug 화면에만 붉게 남긴다.
        //
        // stamp 는 경로 생성 시각이 아니라 근거가 된 영상의
        // 시각이다. 수신측이 지연을 보상할 수 있어야 한다.
        // ========================================================

        publishLanePath(
            lane_path,
            msg->header.stamp
        );


        if (draw_path_overlay_)
        {
            drawPathOverlay(
                lane_path,
                debug_image
            );
        }


        // ========================================================
        // 17. Base Point Visualization
        // ========================================================

        if (left_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    left_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    255,
                    0,
                    0
                ),
                -1
            );
        }


        if (yellow_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    yellow_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    0,
                    255,
                    255
                ),
                -1
            );
        }


        if (right_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    right_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    0,
                    0,
                    255
                ),
                -1
            );
        }


        // ========================================================
        // 18. Image Center
        // ========================================================

        cv::line(
            debug_image,
            cv::Point(
                debug_image.cols / 2,
                0
            ),
            cv::Point(
                debug_image.cols / 2,
                debug_image.rows
            ),
            cv::Scalar(
                255,
                255,
                255
            ),
            1
        );


        // ========================================================
        // 19. Status
        // ========================================================

        // 어디서 끊기는지 보이도록 창 검출 수까지 표시한다.
        //
        //   n/9  : 실제로 n 개 창에서 픽셀을 잡음
        //   REST : 반대편에서 lane_width_px 로 복원
        //   FAIL : 복원도 불가

        const auto lane_label =
            [](const LaneDetectionResult & lane)
            {
                if (!lane.valid)
                {
                    return std::string("FAIL");
                }


                if (lane.found_count == 0)
                {
                    return std::string("REST");
                }


                // 스텝 수는 더 이상 NUM_WINDOWS 로 고정이 아니다.
                // (코너에서는 같은 거리를 더 많은 스텝으로 돈다)
                return std::to_string(lane.found_count);
            };


        const std::string status =
            "L " + lane_label(left_lane) +
            " | Y " + lane_label(yellow_lane) +
            " | R " + lane_label(right_lane);


        // 경로 요약: 길이 / 최대 곡률 반경 / 횡오차
        char path_text[128];

        if (lane_path.valid)
        {
            const double r =
                (lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max
                    : 0.0;

            std::snprintf(
                path_text,
                sizeof(path_text),
                "path %.0fcm | R %.0fcm | cte %+.1fcm | "
                "yaw %+.1fdeg",
                lane_path.length_cm,
                r,
                lane_path.cte_cm,
                lane_path.heading_err * 180.0 / M_PI
            );
        }
        else if (lane_path.built)
        {
            std::snprintf(
                path_text,
                sizeof(path_text),
                "REJECT %s | win %d | R %.0fcm",
                gateNameAscii(lane_path.reject),
                lane_path.source_windows,
                (lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max : 0.0
            );
        }
        else
        {
            std::snprintf(
                path_text,
                sizeof(path_text),
                "path --"
            );
        }


        cv::putText(
            debug_image,
            path_text,
            cv::Point(
                10,
                45
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            lane_path.valid
                ? cv::Scalar(255, 0, 255)
                : cv::Scalar(60, 60, 255),
            1
        );


        cv::putText(
            debug_image,
            status,
            cv::Point(
                10,
                25
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(
                255,
                255,
                255
            ),
            1
        );


        // --------------------------------------------------------
        // 중앙선 대비 자차 위치
        //
        // 자차 종축(BEV 중앙 열)을 아래쪽에 짧게 그어 두면,
        // 판정된 횡거리가 화면에서 바로 읽힌다. 차로 개념은
        // 없으므로 순수하게 "중앙선에서 얼마나 떨어져 있는가"
        // 만 보여준다.
        // --------------------------------------------------------

        {
            const int axis_x =
                (bev_dst_points_.size() == 4)
                    ? static_cast<int>(
                          std::lround(
                              0.5 *
                              (bev_dst_points_[0].x +
                               bev_dst_points_[1].x)))
                    : debug_image.cols / 2;


            cv::line(
                debug_image,
                cv::Point(axis_x, debug_image.rows - 1),
                cv::Point(axis_x, debug_image.rows - 30),
                cv::Scalar(200, 200, 200),
                1
            );


            char center_text[128];

            if (center_fix.valid)
            {
                std::snprintf(
                    center_text,
                    sizeof(center_text),
                    "CENTER off %+.1fcm | src %d",
                    center_fix.offset_cm,
                    center_fix.source
                );
            }
            else
            {
                std::snprintf(
                    center_text,
                    sizeof(center_text),
                    "CENTER --"
                );
            }


            cv::putText(
                debug_image,
                center_text,
                cv::Point(
                    10,
                    65
                ),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                center_fix.valid
                    ? cv::Scalar(0, 255, 0)
                    : cv::Scalar(120, 120, 120),
                1
            );
        }


        // ========================================================
        // 20. Publish Debug Image
        // ========================================================

        auto debug_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                debug_image
            ).toImageMsg();


        debug_image_publisher_->publish(
            *debug_msg
        );


        // ========================================================
        // 21. Log
        // ========================================================

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Sliding Window | %s | %s",
            status.c_str(),
            path_text
        );


        // ========================================================
        // 21-b. 상태 토픽
        //
        // 로그 파싱 대신 기계가 읽을 수 있는 형태로 낸다.
        //   lw/yw/rw : 실제로 픽셀을 잡은 창 수 (복원선은 0)
        //   lv/yv/rv : 그 차선을 쓸 수 있는 상태인지
        //   rad      : 경로 최소 곡률반경 [cm]. 0 이면 직선
        //   coff     : 중앙선 -> 자차 횡거리 [cm]. 오른쪽이 +
        //   csrc     : coff 판정 근거 (0 없음 / 1 노란선 /
        //              2 흰선 두 개 / 3 흰선 하나+맵 단면 /
        //              4 직전 유지)
        // ========================================================

        {
            char rec[320];

            // 제어점 최대 횡편차. 게이트3(시야이탈)이 보는 값 그대로.
            double max_lat = 0.0;

            for (const kau::bezier::Point2 & cp : lane_path.ctrl)
            {
                max_lat =
                    std::max(
                        max_lat,
                        std::abs(cp.y - path_y_offset_cm_));
            }


            const double radius =
                (lane_path.valid && lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max
                    : 0.0;

            std::snprintf(
                rec,
                sizeof(rec),
                "lw=%d yw=%d rw=%d lv=%d yv=%d rv=%d "
                "path=%d gate=%d win=%d band=%.2f "
                "len=%.1f rad=%.1f maxlat=%.1f cte=%.2f yaw=%.2f "
                "coff=%.1f csrc=%d",
                left_lane.found_count,
                yellow_lane.found_count,
                right_lane.found_count,
                left_lane.valid   ? 1 : 0,
                yellow_lane.valid ? 1 : 0,
                right_lane.valid  ? 1 : 0,
                lane_path.valid   ? 1 : 0,
                lane_path.reject,
                lane_path.source_windows,
                yellow_band,
                lane_path.length_cm,
                radius,
                max_lat,
                lane_path.cte_cm,
                lane_path.heading_err * 180.0 / M_PI,
                center_fix.offset_cm,
                center_fix.source
            );

            std_msgs::msg::String status_msg;

            status_msg.data = rec;

            status_publisher_->publish(status_msg);
        }


        // ========================================================
        // 22. 제어점 로그
        //
        // 발행 대상 그 자체. 소비자가 받는 값과 동일하다.
        // ========================================================

        if (lane_path.valid)
        {
            std::string ctrl_text;


            for (
                std::size_t i = 0;
                i < lane_path.ctrl.size();
                ++i
            )
            {
                char one[64];

                std::snprintf(
                    one,
                    sizeof(one),
                    "%s(%.1f, %.1f)",
                    i ? " " : "",
                    lane_path.ctrl[i].x,
                    lane_path.ctrl[i].y
                );

                ctrl_text += one;
            }


            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "quintic Bezier ctrl [cm] %s",
                ctrl_text.c_str()
            );
        }
    }


    // ============================================================
    // cv_bridge Exception
    // ============================================================

    catch (const cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what()
        );
    }


    // ============================================================
    // OpenCV Exception
    // ============================================================

    catch (const cv::Exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "OpenCV exception: %s",
            e.what()
        );
    }


    // ============================================================
    // Standard Exception
    // ============================================================

    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Exception: %s",
            e.what()
        );
    }
}