// ====================================================================
// kau_lane_detection_node.hpp
//
// 클래스 선언만. 구현은 kau_lane_detection.cpp, 진입점은
// kau_lane_detection_node.cpp 에 있다.
// ====================================================================

#ifndef KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_
#define KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>

// 팀 공용 함수형 경로 (docs/경로_형식.md 9장)
#include <kau_msgs/msg/kau_path.hpp>

// RViz 는 custom message 를 못 그린다. 시각화 전용 이산화 발행 (9.9)
#include <nav_msgs/msg/path.hpp>

#include <opencv2/opencv.hpp>

#include "kau_lane_detection/bezier.hpp"

// 이 맵의 차선 정보 (전역 선언). 차로 수, 도로 단면의 선 목록,
// 각 선의 중앙선 대비 횡거리와 색. 관측 하나를 단면의 한 자리에
// 앉히려면 이것이 먼저 있어야 한다.
#include "kau_lane_detection/road_map.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>


class KauLaneDetectionNode : public rclcpp::Node
{
public:

    // ================================================================
    // Lane Detection Result
    // ================================================================

    struct LaneDetectionResult
    {
        // 방향성 창 추적이 잡은 차선 중심점.
        // 차량에 가까운 쪽부터 전방으로 순서대로. [BEV px]
        //
        // 예전에는 창 y 격자에 묶인 x 배열이었다. 그 표현은
        // x = f(y) 를 전제하므로 90도 코너처럼 차선이 가로로
        // 누우면 표현 자체가 불가능하다. 순서 있는 점열은
        // 방향에 무관하다.
        std::vector<cv::Point2d> track_px;

        // 실제로 픽셀을 잡은 스텝 수 (= track_px.size()).
        int found_count = 0;

        bool detected = false;

        // track_px 를 중심선 합성에 쓸 수 있는 상태
        bool valid = false;
    };


    // 생성자. 파라미터 선언 + publisher/subscriber 생성.
    KauLaneDetectionNode();


private:

    // ================================================================
    // Sliding Window Parameters
    //
    // 첨부한 코드의 구조를 기준으로 설정
    // nwindows = 9
    // margin   = 20
    // minpix   = 5
    // ================================================================

    static constexpr int NUM_WINDOWS = 9;

    static constexpr int MIN_VALID_WINDOWS = 3;


    // 가중 최소제곱 다항식 적합 v = f(t). Vandermonde + DECOMP_QR.
    // weights 가 비어 있으면 균등 가중.
    static bool polyFitW(
        const std::vector<double> & ts,
        const std::vector<double> & vs,
        const std::vector<double> & weights,
        int order,
        std::vector<double> & coeffs);


    // 폴리라인 위로 투영. 호길이 s 와 거리 d 를 돌려준다.
    // 양 끝 구간에서는 바깥으로 외삽을 허용한다 (s < 0 가능).
    static void projectOnPolyline(
        const std::vector<cv::Point2d> & poly,
        const std::vector<double> & cum,
        const cv::Point2d & p,
        double & s_out,
        double & d_out);


    // pts 각 점의 ref 기준 부호 있는 횡거리 [px]. 차량 오른쪽이 +.
    static std::vector<double> lateralOffsets(
        const std::vector<cv::Point2d> & ref,
        const std::vector<cv::Point2d> & pts);


    // 위 값의 중앙값. 차선이 통째로 어느 쪽에 있는지 판정용.
    static double medianLateralOffset(
        const std::vector<cv::Point2d> & ref,
        const std::vector<cv::Point2d> & pts);


    // ref 기준 회랑 [lo, hi] * lane_width 를 벗어나는 순간 잘라낸다.
    // 추적이 체커보드 연석이나 다른 구간 차선으로 갈아탄 지점에서
    // 끊는다. 통째로 버리지 않고 정상이던 앞부분은 남긴다.
    static std::vector<cv::Point2d> truncateAtCorridor(
        const std::vector<cv::Point2d> & track,
        const std::vector<cv::Point2d> & ref,
        double want_sign,
        double lo_px,
        double hi_px);


    // 순서 있는 점열을 국소 법선(차량 오른쪽) 방향으로 평행이동.
    // 90도 코너에서는 x 방향 offset 이 틀리므로 반드시 법선이어야 한다.
    static std::vector<cv::Point2d> offsetTrack(
        const std::vector<cv::Point2d> & pts,
        double offset_px);


    // 파라미터(int 배열) -> cv::Scalar.
    static cv::Scalar toScalar(
        const std::vector<int64_t> & v);


    // Horner 평가.
    static double polyEval(
        const std::vector<double> & coeffs,
        double y);


    // src 사다리꼴 -> dst 사각형 변환행렬 생성.
    void updateBevGeometry();


    // 파라미터 변화 감지. 바뀐 프레임에만 행렬 재계산.
    void refreshBevParameters();


    // CameraInfo 수신 -> K, D.
    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);


    // BEV 하단 히스토그램 peak. 슬라이딩 윈도우 시작점.
    // band_used 에 실제로 쓰인 밴드 비율을 돌려준다(폴백 여부 확인용).
    int findHistogramPeak(
        const cv::Mat & mask,
        int x_start,
        int x_end,
        double * band_used = nullptr);


    // 방향성 슬라이딩 윈도우 추적.
    //
    // 창이 항상 위(-y)로만 올라가는 대신, 지금까지의 진행 방향
    // theta 로 회전하며 그 방향으로 전진한다. 차선이 가로로
    // 누워도 창이 차선을 따라 돌아간다.
    LaneDetectionResult detectLaneSlidingWindow(
        const cv::Mat & mask,
        int base_x,
        cv::Mat & debug_image,
        const cv::Scalar & window_color);


    // ================================================================
    // 공통 경로 형식 — quintic Bezier 제어점
    //
    // docs/경로_형식.md 규약
    //
    //   1. 경로는 제어점으로만 생성/계산/출력. 이산 좌표 금지.
    //   2. 발행 차수는 항상 5.
    //   3. 내부 적합 차수는 달라도 되며, 낮은 차수로 적합했다면
    //      degree elevation 으로 5차로 승격해 발행 (무손실).
    //
    // 여기서 내부 적합은 매개변수 s (누적 호길이) 에 대한
    // x(s), y(s) 이고 차수는 최대 3차다. 이유:
    //
    //   x = f(y) 형태는 차선이 가로로 누우면 dx/dy = 무한대라
    //   표현 자체가 불가능하다. 90도 코너가 통째로 끊기던
    //   원인이었다. s 로 매개화하면 방향 전제가 사라진다.
    //
    //   90도 코너는 s 에 대한 사분원이라 2차로는 모자라고
    //   3차면 충분하다. 3차 매개곡선은 t 에 대해서도 정확히
    //   3차이므로, 멱기저 -> Bernstein -> degree elevation 이
    //   전부 정확한 기저 변환이다. 근사가 한 번도 개입하지 않는다.
    //
    // 이 경로는 차로를 고르지 않는다. 아래 sources (buildCenterlinePath
    // 참고) 는 왼쪽 흰선/노란 중앙선/오른쪽 흰선 세 후보를 전부
    // "중앙선의 위치 추정치"로 환산해 가중합하므로, 애초에 자차가
    // 어느 차로에 있는지 묻지 않고 중앙선만 추종한다.
    // ================================================================


    struct LanePath
    {
        // 제어점 6개 [cm]. 이것이 발행 대상의 전부다.
        kau::bezier::Ctrl ctrl;

        bool valid = false;

        // 아래는 전부 진단용 (발행하지 않음)
        double length_cm = 0.0;

        double kappa_max = 0.0;

        // 실관측 구간 [cm]. 근거 점이 끊기지 않고 이어지는
        // 시점부터의 호길이. 그 뒤는 외삽이다. KauPath 로 발행한다.
        double valid_length_cm = 0.0;

        // 0.0 ~ 1.0. 근거량 x 관측 비율.
        double confidence = 0.0;

        double cte_cm = 0.0;

        double heading_err = 0.0;

        int source_windows = 0;

        // 제어점 생성까지는 끝난 상태. 게이트 통과 여부와 무관하다.
        // 기각된 경로도 debug 화면에는 그려서 무엇이 막혔는지 보인다.
        bool built = false;

        // 게이트 기각 사유. 0 = 통과
        //   1 근거부족  2 조향한계  3 시야이탈
        int reject = 0;
    };


    // ================================================================
    // 차선 정보 — 도로 모형
    //
    // 선언 자체는 road_map.hpp 에 있다 (namespace kau_road).
    // 도로 단면의 선 목록, 각 선의 중앙선 대비 횡거리, 색.
    //
    // 이 코드는 차로를 지정하지 않는다. 자차가 중앙선 왼쪽
    // 차선에 있든 오른쪽 차선에 있든 상관하지 않고, 중앙선
    // (황색 점선)을 그대로 추종 목표로 삼는다. 그래서 road_map.hpp
    // 에는 "차로 번호"나 "지정 차선" 개념이 없다.
    //
    // 그래도 단면 선언 자체는 필요하다. 선이 하나만 보일 때
    // 그것이 어느 선인지 고를 근거가 있어야 하기 때문이다 —
    // 흰선 하나만 잡혔을 때 그게 왼쪽 도로 경계인지 오른쪽
    // 도로 경계인지 알아야, 거기서 중앙선이 어느 쪽으로
    // 31.25 cm 떨어져 있는지 역산할 수 있다 (identifyLine).
    //
    // 맵 mesh 실측 (custom_71e69ee938032295503bfed557fde18c.dae,
    // 삼각형을 x = 6.0 m 평면으로 잘라 단면을 낸 값. 단위 m):
    //
    //   road   y 0.8805 .. 1.5805    아스팔트 폭 0.700
    //   ol     y 0.8805 .. 0.9555    흰 실선   폭 0.075
    //   cl     y 1.2180 .. 1.2430    황색 점선 폭 0.025
    //   il     y 1.5055 .. 1.5805    흰 실선   폭 0.075
    //
    //   중앙선 중심 <-> 흰선 중심 = 31.25 cm   (= 차로 폭)
    //   흰선 중심   <-> 흰선 중심 = 62.50 cm   (= 도로 폭)
    //
    // 도로는 닫힌 고리다. 같은 x = 6.0 평면이 도로를 두 번 지나고
    // 두 통과의 선 순서가 서로 뒤집혀 있다 (아래쪽 ol-cl-il,
    // 위쪽 il-cl-ol). il 이 양쪽에서 안쪽을 향하므로 ol 이 바깥
    // 경계, il 이 안쪽 경계인 무한 루프다.
    //
    // 흰선은 차선 경계가 아니라 도로의 양 경계이고, 도로 안쪽의
    // 유일한 경계가 중앙선이다. 두 흰선 중 어느 쪽이 트랙 바깥
    // 경계인지는 루프를 도는 방향에 달렸고, 검출에는 쓰이지
    // 않는다 (차량 기준 좌/우만 쓴다).
    //
    // 같은 값을 영상에서도 확인했다. 직선 구간 한 프레임을
    // undistort 해 지면으로 역투영하면 (h = 14.65 cm)
    //
    //   도로 바깥 가장자리 간격 70.0 cm  ->  h 14.552
    //   흰선 안쪽 가장자리 간격 55.0 cm  ->  h 14.670
    //   중앙선 <-> 왼쪽 흰선 안쪽 27.5 cm -> h 14.708
    //   중앙선 <-> 오른쪽 흰선 안쪽 27.5 cm -> h 14.633
    //
    // 같은 프레임의 BEV 에서 흰선 사이 간격이 195.0 px 이고
    // sx = 0.3220 cm/px 이므로 62.79 cm. 실측 62.50 과 0.5% 차이.
    // ================================================================


    // 중앙선 대비 자차 위치의 최근 실측 (진단 + identifyLine prior 용).
    //
    // 예전에는 이 값으로 "차로 번호"(0/1)와 "지정 차선 이탈 여부"를
    // 판정해 EgoLane 으로 발행했다. 이제 차로 개념이 없으므로 그
    // 판정은 없앴다. 남은 건 순수하게 "중앙선에서 자차가 얼마나
    // 떨어져 있는가" 라는 진단값과, identifyLine 이 흰선 하나를
    // 왼쪽/오른쪽 경계 중 어디에 앉힐지 고를 때 쓰는 기준점뿐이다.
    struct CenterFix
    {
        bool valid = false;

        // 중앙선 -> 자차 부호 있는 횡거리 [cm]. 차량 오른쪽이 +.
        double offset_cm = 0.0;

        // 판정 근거
        //   0 없음
        //   1 노란 중앙선 실측
        //   2 흰선 두 개 사이를 반으로 갈라 역산
        //   3 흰선 하나 + 맵 단면으로 역산
        //   4 직전 프레임 유지 (이번 프레임 근거 없음)
        int source = 0;
    };


    // 기준선 대비 부호 있는 횡거리 하나. 차량 오른쪽이 +.
    //
    // lateralOffsets 는 투영 t 를 [0,1] 로 자르지만, 자차 위치는
    // 추적 시작점보다 뒤(BEV 아래쪽)에 있어서 그대로 쓰면 첫 점
    // 까지의 직선거리가 나와 값이 부풀려진다. 양 끝 구간에서는
    // 외삽을 허용한다.
    static double lateralOffsetAt(
        const std::vector<cv::Point2d> & ref,
        const cv::Point2d & p);


    // 중앙선 대비 자차 횡거리를 갱신한다.
    //
    // 반드시 흰선 검증(16-a) 뒤, 소실 복원(16-c) 앞에서 부른다.
    // 복원된 차선은 관측이 아니라 "차로 폭만큼 옆에 있을 것"
    // 이라는 가정이므로, 그것으로 이 값을 재면 가정을 관측
    // 처럼 되돌려 읽는 순환이 된다.
    //
    // center_fix_ 이력(유지 프레임 수)을 갱신하고, 확실한 근거
    // (노란선 실측 또는 흰선 두 개)일 때만 centerOffsetPriorCm()
    // 의 prior 도 함께 갱신한다.
    CenterFix resolveCenterOffset(
        const LaneDetectionResult & left,
        const LaneDetectionResult & yellow,
        const LaneDetectionResult & right,
        int bev_height);


    // BEV 상의 자차 종축 폴리라인 (아래 -> 위, 진행방향).
    // 관측선이 자차 기준 어느 쪽에 얼마나 있는지 재는 기준선.
    std::vector<cv::Point2d> egoAxis(int bev_height) const;


    // 관측선 하나를 도로 단면(road_map.hpp 의 LINES)의 한 자리에
    // 앉힌다. 반환값은 LINES 의 index, 못 고르면 -1.
    //
    // 선이 하나만 보일 때 그것이 왼쪽 가장자리인지 오른쪽
    // 가장자리인지는 관측만으로는 알 수 없다. 히스토그램이
    // 화면 반쪽 중 어디서 찾았는지는 근거가 못 된다. 코너에서
    // 차선이 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
    //
    // 맵 단면이 있으면 답이 나온다. 자차가 중앙선에서 몇 cm
    // 떨어져 있는지(ego_offset_cm) 를 알면 각 선이 자차 종축
    // 에서 몇 cm 떨어져 있어야 하는지가 정해지고, 실측 횡거리를
    // 거기에 맞추면 된다. 이 맵에서 두 흰선의 기대 위치는
    // 62.5cm 떨어져 있으므로 판정 여유가 넉넉하다.
    //
    // 기대 위치의 기준점 ego_offset_cm 은 centerOffsetPriorCm() 이
    // 준다. 노란선/흰선 두 개로 잰 값이 싱싱하면 그것을 쓰고,
    // 없으면 중앙선 위에 있다고 둔다 (차로 개념이 없으므로 이
    // 값이 유일하게 근거 없는 기본값이다). 흰선 하나로 얻은
    // 값은 들어가지 않는다 (자기 출력 되먹임 방지).
    int identifyLine(
        const std::vector<cv::Point2d> & track,
        kau_road::LineType type,
        double ego_offset_cm,
        double sx,
        int bev_height) const;


    // identifyLine 에 넣을 "자차가 중앙선에서 몇 cm 에 있는가".
    //
    // 노란선이나 흰선 두 개로 잰 값이 아직 싱싱하면 그것을 쓰고,
    // 없으면 중앙선 위에 있다고 둔다(0.0). 차로 개념이 없으므로
    // "지정 차선 중심" 같은 기본값은 없다 — 중앙선 자체가 목표이니
    // 근거가 없을 때의 최선의 가정도 중앙선이다.
    //
    // 흰선 하나로 얻은 값(source 3)은 절대 여기 들어가지 않는다.
    // 그 값은 이 prior 로 자리를 골라서 나온 것이라, 되먹이면
    // 오차가 스스로를 키운다.
    double centerOffsetPriorCm() const;


    // 발행 게이트 (docs/경로_형식.md 와 무관한 안전장치).
    // 통과면 0, 아니면 기각 코드.
    int pathGate(const LanePath & path, double sx) const;

    static const char * gateName(int code);

    // cv::putText 는 CJK 글리프가 없어 물음표로 나온다.
    // debug 화면용 ASCII 이름.
    static const char * gateNameAscii(int code);

    // lane_width_cm + 카메라 기하 -> lane_width_px_ 재계산.
    // CameraInfo 수신 시와 BEV 기하 변경 시 호출한다.
    void refreshDerivedScale();


    // 제어점 -> KauPath 적재 후 발행 (docs/경로_형식.md 9.5).
    // 게이트를 통과한 경로만 발행한다.
    void publishLanePath(
        const LanePath & path,
        const rclcpp::Time & stamp);


    // BEV 축척 [cm/px]. 세로는 intrinsic + 소실점에서 유도.
    bool bevScale(
        double & sx,
        double & sy,
        double & x_base_cm) const;


    // 차선 중심선 -> quintic Bezier 제어점 (공통 경로 형식).
    LanePath buildCenterlinePath(
        const LaneDetectionResult & left,
        const LaneDetectionResult & yellow,
        const LaneDetectionResult & right,
        int bev_width,
        int bev_height,
        std::vector<cv::Point2d> & center_pts_px);


    // 발행한 Bezier 를 BEV 위에 되돌려 그린다 (시각화 전용).
    void drawPathOverlay(
        const LanePath & path,
        cv::Mat & image);


    // 메인 파이프라인.
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg);


    // ================================================================
    // Subscribers
    // ================================================================

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        image_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscriber_;


    // ================================================================
    // Publishers
    // ================================================================

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        undistorted_image_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        bev_image_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        hls_binary_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        combined_binary_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        debug_image_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        roi_overlay_publisher_;

    // 검출 상태 진단. 실패 프레임 수집 스크립트가 이걸 보고 판정한다.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        status_publisher_;

    // 이 노드의 최종 산출물. /lane/center, RELIABLE depth 1 (9.9)
    rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr
        lane_path_publisher_;

    // RViz 전용. BEST_EFFORT. 제어 경로는 여기 의존하지 않는다.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        viz_path_publisher_;


    // ================================================================
    // BEV Parameters
    // ================================================================

    double bev_src_center_x_;

    double bev_src_top_y_;

    double bev_src_bottom_y_;

    double bev_vanishing_y_;

    // 파라미터가 아니라 소실점에서 계산되는 값
    double bev_src_top_width_ = 0.0;

    double bev_src_bottom_width_;

    double bev_dst_margin_ratio_;

    int bev_out_width_;

    int bev_out_height_;

    int window_margin_;

    int min_pixels_;

    // 방향성 추적: 한 스텝 전진 거리 [BEV px].
    // 코너 추종 능력을 지배하는 값이다.
    double track_step_px_;

    // 방향성 추적: 한 스텝에서 허용하는 최대 방향 변화 [deg]
    double track_max_turn_deg_;

    // 방향성 추적: 최대 스텝 수 (곡선은 직선보다 스텝이 더 필요)
    int track_max_steps_;

    // 방향성 추적: 연속으로 비어도 관성 주행할 스텝 수 (점선 대응)
    int track_max_miss_;

    // 흰선이 노란선 기준으로 제 쪽에 있는지 판정 [lane_width_px 비율].
    // 이보다 가까우면 반대쪽 선을 문 것으로 보고 버린다.
    double lane_side_min_ratio_;

    // 흰선 회랑 [lane_width 비율]. 이 밖으로 나가면 그 지점에서 절단.
    double lane_corridor_lo_;

    double lane_corridor_hi_;

    // 중심선 후보 이탈 판정 [lane_width_px 비율].
    // 기준선에서 이만큼 이상 떨어진 후보는 다른 선을 문 것으로 본다.
    double center_outlier_ratio_;

    std::vector<int64_t> yellow_hls_lo_;

    std::vector<int64_t> yellow_hls_hi_;

    std::vector<int64_t> white_hls_lo_;

    std::vector<int64_t> white_hls_hi_;

    // 차로 폭 [BEV px]. 파라미터가 아니라 lane_width_cm 과
    // 카메라 기하에서 유도한 값 (refreshDerivedScale 참고)
    double lane_width_px_;

    // 히스토그램 밴드 높이 (BEV 하단에서 차지하는 비율)
    double histogram_band_ratio_;

    bool publish_roi_overlay_;


    // ================================================================
    // 경로 발행 파라미터 (docs/경로_형식.md)
    // ================================================================

    std::string path_frame_id_;

    bool publish_path_;

    double lane_width_cm_;

    double camera_height_cm_;

    double path_x_offset_cm_;

    double path_y_offset_cm_;

    bool draw_path_overlay_;


    // ================================================================
    // 중앙선 대비 자차 위치 (진단 + identifyLine prior)
    //
    // 차로 번호 개념은 없다. 남은 건 "중앙선에서 자차가 얼마나
    // 떨어져 있는가" 라는 스칼라값 하나뿐이다.
    // ================================================================

    // 근거가 없어도 직전 판정을 유지할 프레임 수
    int center_hold_frames_;

    // 판정 이력
    CenterFix center_fix_;

    int center_fix_miss_ = 0;

    // 확실한 근거(노란선 또는 흰선 두 개)로 마지막에 잰
    // 중앙선 대비 자차 횡거리 [cm]. identifyLine 의 기준점.
    double center_prior_offset_cm_ = 0.0;

    // 위 값이 몇 프레임 전 것인지. 오래되면 안 쓴다.
    int center_prior_age_ = 1000000;


    // ================================================================
    // 발행 게이트
    // ================================================================

    bool path_gate_enable_;

    int min_path_windows_;

    double wheelbase_cm_;

    double max_steer_deg_;

    double max_lateral_cm_;


    std::vector<cv::Point2f> bev_src_points_;

    std::vector<cv::Point2f> bev_dst_points_;

    cv::Mat perspective_matrix_;


    // ================================================================
    // Camera Calibration
    // ================================================================

    cv::Mat camera_matrix_;

    cv::Mat distortion_coefficients_;

    bool camera_calibrated_;
};

#endif  // KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_