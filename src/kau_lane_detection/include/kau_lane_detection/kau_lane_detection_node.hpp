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
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

// Pan 명령 (/camera/pan, 절대각 rad)
#include <std_msgs/msg/float64.hpp>

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


    // ================================================================
    // Pan 기반 Lane Lost Recovery
    //
    // 한쪽 흰선이 연속으로 안 잡히면 그 방향으로 카메라를 돌려
    // 찾고, 찾으면(또는 상한까지 못 찾으면) 0 으로 복귀한다.
    //
    // ★ pan != 0 인 프레임의 BEV 좌표는 신뢰하지 않는다.
    //
    //   updateBevGeometry() 의 src 사다리꼴은 "카메라가 정면을
    //   본다" 를 가정한 고정값이다. 카메라가 돌아가면 그 가정이
    //   깨져 BEV 좌표와 base_link 의 대응이 어긋난다. tf2 로
    //   base_link <-> camera_optical_frame 을 매 프레임 조회해
    //   행렬을 다시 세우는 동적 BEV 는 아직 없다.
    //
    //   그래서 Idle 이 아닌 동안에는 imageCallback 이 경로를
    //   새로 짓지 않는다 (/lane/center 가 그동안 끊긴다).
    // ================================================================

    enum class PanSearchState
    {
        Idle,       // pan = 0. 소실 감시만 한다
        Searching,  // 소실된 쪽으로 전진하며 재검출을 노린다
        Holding,    // 재검출됨. 직선 구간이 나올 때까지 그 각도를 유지
        Returning   // 0 으로 복귀 중
    };


    // 순서 있는 점열이 얼마나 휘었는가 [deg]. 판정 불가면 음수.
    //
    // 앞 절반의 현(chord)과 뒤 절반의 현이 이루는 각이다.
    //
    // BEV 좌표로 재도 된다. 호모그래피는 직선을 직선으로 보내므로
    // "휘었나 안 휘었나" 는 카메라가 돌아가 BEV 가정이 깨진
    // 상태에서도 그대로 읽힌다. 다만 각도의 크기는 보존되지
    // 않으므로(원근에 따라 늘거나 준다) 이 값은 세상의 도(度)가
    // 아니라 BEV 영상 위의 도다. 임계값은 실측으로 잡을 것.
    static double trackBendDeg(
        const std::vector<cv::Point2d> & pts);


    // pan 을 0 으로 되돌렸을 때 이 점들 중 몇 개가 여전히
    // 화면 안에 들어오는가. 계산 불가면 -1.
    //
    // ------------------------------------------------------------
    // 복귀 조건이 왜 "직선 구간" 이 아니라 이것인가
    //
    // 예전에는 도로가 펴지면 복귀했다. 그런데 "도로가 곧다" 와
    // "정면으로 돌아가도 그 선이 보인다" 는 다른 명제다.
    //
    // 실측: 정지 상태의 곧은 구간에서 왼쪽 흰선이 화면 밖에
    // 있었다. 도로는 곧으니 직선 판정은 통과하고, 복귀하면
    // 그 선은 여전히 안 보이니 즉시 재탐색이 걸렸다. 15초에
    // 9번 왕복했고 그동안 /lane/center 가 계속 끊겼다.
    // 직선 임계값을 아무리 조여도 멎지 않는 종류의 문제다.
    //
    // 그래서 목적을 직접 잰다 — 복귀해도 그 선을 다시 잡을
    // 수 있는가.
    //
    // ------------------------------------------------------------
    // 기하
    //
    // BEV -> 원본(undistort) 영상 역호모그래피로 추적점의 영상
    // 열 u 를 복원하면 영상각이 나온다 (오른쪽이 +):
    //
    //     a = atan((u - cx) / fx)
    //
    // 카메라가 pan 각 theta (왼쪽 +) 로 돌아가 있으므로 그 점의
    // 차량 기준 방위는 psi = theta - a 이고, pan = 0 일 때의
    // 영상각은
    //
    //     a0 = -psi = a - theta
    //
    // 이 값이 화면 반각(여유 제외) 안에 들면 정면에서도 보인다.
    //
    // 역호모그래피는 원본 영상과 BEV 사이의 순수 2D 사상이라
    // pan 때문에 깨진 평면 가정과 무관하게 정확하다. 깨지는
    // 것은 BEV 좌표의 "미터 해석" 이지 픽셀 대응이 아니다.
    // ------------------------------------------------------------
    int countVisibleAtZeroPan(
        const std::vector<cv::Point2d> & track_px,
        double pan_deg) const;


    // camera_pan_joint 실측 각도 수신 (50 Hz).
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg);


    // 최신 LaserScan 보관 (장애물 가림 판정용).
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg);


    // 소실된 쪽에 차선을 가릴 만한 장애물이 있는가.
    //
    // dir 은 pan_search_dir_ 와 같은 규약 (+1 왼쪽 / -1 오른쪽).
    //
    // 차선이 화면에서 사라지는 이유는 두 가지다. 코너를 돌면서
    // 시야 밖으로 나갔거나, 앞의 장애물에 가렸거나. 앞의 경우는
    // 고개를 돌리면 다시 보이지만, 뒤의 경우는 아무리 돌려도
    // 안 보인다 — 가린 물체가 같이 따라오기 때문이다. 그래도
    // 탐색을 걸면 상한까지 헛돌고 그동안 경로만 끊긴다.
    //
    // 라이다는 그 둘을 구분할 수 있다. 소실된 쪽 전방 부채꼴에
    // 가까운 반사가 있으면 가림으로 보고 탐색을 걸지 않는다.
    //
    // /scan_filtered 를 직접 본다. kau_object_detection 의
    // /perception/obstacles 는 map 프레임이라 map->base_link TF
    // 가 필요한데, 이 노드는 측위 비의존이 대전제라 쓸 수 없다.
    // (게다가 그 노드는 Gazebo 참값 pose 에 의존해 실기에서는
    //  빈 배열을 낸다.) LaserScan 은 lidar_link 프레임이고
    // lidar_link -> base_link 는 URDF 고정 변환(rpy 0 0 0)이라
    // 방위각이 그대로 통한다. 종방향 2.7 cm 차이는 부채꼴
    // 판정에서 무시한다.
    //
    // 나이는 node clock 이 아니라 frame_stamp (지금 처리 중인
    // 영상의 시각) 기준으로 잰다. 영상과 scan 은 같은 출처에서
    // 스탬프를 받으므로(시뮬은 Gazebo, 실기는 시스템 클록) 이
    // 비교는 use_sim_time 설정과 무관하게 성립한다. node clock
    // 으로 재면 use_sim_time=false 인데 센서가 sim time 을 달고
    // 오는 순간 나이가 1e9 초로 나와 판정이 영영 죽는다.
    //
    // scan 이 없거나 오래됐으면 false 를 돌려준다 — 판단 근거가
    // 없을 때 탐색을 막지는 않는다 (fail-open).
    bool obstacleOnSide(
        int dir,
        const rclcpp::Time & frame_stamp) const;


    // 절대각 [deg] 을 pan_max_deg_ 로 클램프해 /camera/pan 에
    // rad 로 발행하고 pan_cmd_deg_ 를 갱신한다.
    //
    // 한때 목표/발행을 분리해 매 프레임 각속도 제한으로 다가가는
    // 연속 램프를 썼지만, 실기에서 "꺾다 말다" 하는 움직임이
    // 나와 이산 스텝으로 되돌렸다.
    void commandPan(double cmd_deg);


    // 좌/우 원본 검출 상태를 보고 위 상태기계를 한 프레임 굴린다.
    // 반드시 흰선 검증(16-a) 뒤, 소실 복원(16-c) 과 무관한 시점에
    // 부른다 (복원선은 found_count 가 0 이라 어느 쪽이든 같다).
    // frame_stamp 는 지금 처리 중인 영상의 시각이다. 각속도
    // 제한의 dt 와 Holding 경과시간, 장애물 판정의 scan 나이를
    // 전부 이 시계로 잰다. node clock 을 쓰면 use_sim_time 설정과
    // RTF 에 따라 전부 어긋난다.
    void updatePanSearch(
        const LaneDetectionResult & left,
        const LaneDetectionResult & yellow,
        const LaneDetectionResult & right,
        const rclcpp::Time & frame_stamp);


    // ================================================================
    // 시간축 강건화 (EMA)
    //
    // 게이트는 프레임 하나만 보고 통과/기각을 정한다. 임계값을
    // 조이면 멀쩡한 차선이 떨어져 나가고, 풀면 도로 밖 노란
    // 물체(원경의 그림 등)가 통과한다. 어느 쪽이든 하류
    // state machine 의 fail-safe 판정이 흔들린다.
    //
    // 프레임 하나로 정할 수 없는 문제이므로 시간축을 쓴다.
    // 게이트를 통과한 관측을 EMA 추정치에 섞되,
    //
    //   1. 섞는 양(alpha)을 그 프레임의 confidence 에 비례시킨다.
    //      믿기 힘든 프레임은 추정치를 조금밖에 못 움직인다.
    //
    //   2. 추정치에서 path_ema_gate_cm_ 이상 벗어난 관측은
    //      아예 섞지 않고, 직전 추정치를 그대로 내보내면서
    //      confidence 만 떨어뜨린다. 하류는 "경로는 계속 오는데
    //      신뢰도가 미끄러진다" 를 보고 스스로 판단할 수 있다.
    //
    //   3. 같은 주장이 path_ema_relock_frames_ 번 연속 오면
    //      노이즈가 아니라 실제 변화로 보고 통째로 재잠금한다.
    //      (그러지 않으면 진짜 차선 변화에 영원히 눈감는다)
    //
    // 자기운동 보정은 하지 않는다. 이 노드는 측위 비의존이
    // 대전제라 odom/TF 를 볼 수 없다. 차선을 따라가는 동안
    // base_link 기준 경로는 거의 불변이라 편향이 작지만,
    // 고속에서는 추정치가 뒤로 끌리는 지연이 생긴다.
    // ================================================================

    // 제어점 열 사이의 평균 거리 [cm]. 크기가 다르면 음수.
    static double pathDeviationCm(
        const kau::bezier::Ctrl & a,
        const kau::bezier::Ctrl & b);


    // ctrl 이 바뀐 뒤 진단값(길이/곡률/cte/heading)을 다시 맞춘다.
    // EMA 로 섞은 제어점은 관측 당시의 진단값과 더 이상 짝이
    // 맞지 않는다. valid_length 는 하류가 LookAhead 를 자르는
    // 근거이므로 특히 그대로 두면 안 된다.
    static void refreshPathMetrics(LanePath & path);


    // 관측 경로를 EMA 추정치와 합쳐 발행할 경로로 바꾼다.
    // path.valid 가 false 면 관측 없음으로 보고 상태만 늙힌다.
    void robustifyPath(LanePath & path);


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

    // camera_pan_joint 실측 각도. 실기 드라이버는 서보 인코더가
    // 없어 마지막 명령각을 되쏘고, 시뮬은 Gazebo 실측 관절각이다.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscriber_;

    // 장애물 가림 판정용. lidar_link 프레임, 실기/시뮬 공통.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;


    // ================================================================
    // Publishers
    // ================================================================

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        undistorted_image_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        bev_image_publisher_;

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

    // Pan 명령. physicar_driver_node 의 apply_pan() 이 절대각
    // [rad] 으로 받아 ±30 deg 로 다시 클램프한다.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        camera_pan_publisher_;


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


    // ================================================================
    // 시간축 강건화 (EMA) — 파라미터와 상태
    // ================================================================

    bool path_ema_enable_;

    // confidence = 1 일 때의 혼합 비율. 작을수록 무겁고 느리다.
    double path_ema_alpha_;

    // 추정치에서 이만큼(제어점 평균거리) 벗어난 관측은 섞지 않는다.
    // 0 이하면 게이트 없이 순수 EMA.
    double path_ema_gate_cm_;

    // 기각이 이만큼 연속되면 실제 변화로 보고 재잠금한다.
    int path_ema_relock_frames_;

    // 관측이 이만큼 연속으로 없으면 추정치를 폐기한다.
    int path_ema_reset_frames_;


    kau::bezier::Ctrl path_ema_ctrl_;

    bool path_ema_valid_ = false;

    double conf_ema_ = 0.0;

    double valid_len_ema_ = 0.0;

    int path_ema_reject_streak_ = 0;

    // 직전 프레임의 관측-추정치 편차 [cm]. 진단 전용
    // (path_ema_gate_cm 을 실측으로 잡는 근거가 된다).
    double last_path_dev_cm_ = -1.0;

    int ema_miss_streak_ = 0;


    // ================================================================
    // 장애물 가림 판정 (LaserScan)
    // ================================================================

    bool obstacle_pan_block_enable_;

    // 소실된 쪽 부채꼴 [deg]. 차량 정면이 0, 왼쪽이 +.
    //
    // 흰선은 중앙선에서 31.25 cm 옆이고 카메라는 1 m 안팎을
    // 본다. 그 선을 가리는 물체의 방위각이 대략 이 범위에 든다.
    double obstacle_sector_min_deg_;

    double obstacle_sector_max_deg_;

    // 이 거리 안의 반사만 가림으로 본다 [m].
    double obstacle_max_range_m_;

    // 단일 빔 노이즈를 배제하기 위한 최소 점 수.
    int obstacle_min_points_;

    // scan 이 이보다 오래되면 판단하지 않는다 [s].
    double obstacle_scan_timeout_s_;


    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;


    // ================================================================
    // Pan 탐색 상태
    // ================================================================

    bool pan_search_enable_;

    // 탐색 시작 각도 [deg]. 첫 명령은 이 각도로 한 번에 간다.
    //
    // 차선이 화면에서 빠지는 상황은 대개 코너라 몇 도로는
    // 다시 안 들어온다. 3도씩 기어가면 재검출까지 프레임을
    // 낭비하고 그동안 경로도 끊겨 있다.
    double pan_start_deg_;

    // 시작 각도에서 못 찾았을 때의 한 스텝 회전량 [deg].
    double pan_step_deg_;

    double pan_max_deg_;

    // |실측 - 목표| 가 이 안이면 정착으로 보고 다음 스텝으로 간다.
    double pan_settle_tol_deg_;

    int pan_trigger_miss_frames_;

    // 명령 부호 뒤집개. 카메라가 명령과 반대로 돌면 -1.0.
    double pan_sign_;

    // 재검출 직후 추가로 더 돌릴 각도 [deg].
    //
    // 탐색은 "겨우 보이기 시작한" 지점에서 멈춘다. 그 각도에서는
    // 찾은 선이 화면 가장자리에 걸쳐 있어, 차가 조금만 움직여도
    // 다시 빠진다. 한 번 더 꺾어 안쪽으로 당겨 놓는다.
    // pan_max_deg_ 는 commandPan 이 그대로 지킨다.
    double pan_overshoot_deg_;


    // 재검출 후 복귀 조건 — 직선 구간
    //
    // 찾자마자 복귀하면 코너 한복판에서 원위치로 돌아가 그
    // 선을 곧바로 다시 놓친다. pan 이 왕복하며 그동안 경로가
    // 계속 끊긴다. 그래서 재검출 뒤에는 각도를 유지한 채
    // (Holding) 도로가 다시 펴질 때까지 기다렸다 복귀한다.
    // 화면 가장자리 여유 [deg]. 반각에서 이만큼 뺀 범위만
    // "보인다" 로 친다. 가장자리에 겨우 걸친 선은 차가 조금만
    // 움직여도 다시 빠지기 때문이다.
    double pan_return_fov_margin_deg_;

    int pan_return_confirm_frames_;

    // 직선 판정에 필요한 최소 추적점 수.
    //
    // 짧은 점열은 어떤 코너에서도 직선으로 보인다 (구현부
    // 주석 참고). 근거가 모자라면 "직선 아님" 이 아니라
    // "판정 불가" 로 다뤄 Holding 을 유지한다.
    int pan_return_min_points_;

    // 직선이 끝내 안 나올 때의 탈출구 [s].
    //
    // 0 이하면 무한 대기 — 직선을 만날 때까지 복귀하지 않는다
    // (기본값). Holding 동안에는 경로를 짓지 않으므로 그동안
    // /lane/center 가 계속 끊겨 있다는 뜻이다. 그게 곤란해지면
    // 양수로 바꿔 상한을 준다.
    double pan_hold_timeout_s_;


    PanSearchState pan_state_ = PanSearchState::Idle;

    // 탐색 회전 방향. +1 왼쪽 / -1 오른쪽 / 0 탐색 중 아님.
    // camera_pan_joint 는 axis +Z 라 +rad 이 왼쪽이다.
    int pan_search_dir_ = 0;

    // 표시용. 스텝 방식에서는 발행값을 그대로 따라간다.
    double pan_goal_deg_ = 0.0;

    // 실제로 /camera/pan 에 발행한 각도 [deg]
    double pan_cmd_deg_ = 0.0;

    // /joint_states 가 알려준 현재 각도 [deg]
    double actual_pan_deg_ = 0.0;

    bool joint_state_received_ = false;

    // 좌/우 흰선이 연속으로 안 잡힌 프레임 수
    int left_miss_streak_ = 0;

    int right_miss_streak_ = 0;

    // Holding 에서 연속으로 직선 판정이 난 프레임 수
    int pan_straight_streak_ = 0;

    // 직전 프레임의 꺾임각 [deg]. 진단 전용 (음수 = 판정 불가).
    // 복귀 판정에는 쓰이지 않는다 — status 의 pbend 로만 나간다.
    double last_bend_deg_ = -1.0;

    // 직전 프레임에 "정면에서도 보인다" 로 센 점 수. 진단 전용.
    int last_visible_at_zero_ = -1;

    // Holding 진입 시각. 기본 생성 rclcpp::Time 은 SYSTEM_TIME
    // 이라 node clock(ROS_TIME) 과 빼면 예외가 난다. 명시한다.
    rclcpp::Time pan_hold_start_{0, 0, RCL_ROS_TIME};


    std::vector<cv::Point2f> bev_src_points_;

    std::vector<cv::Point2f> bev_dst_points_;

    cv::Mat perspective_matrix_;


    // ================================================================
    // Camera Calibration
    // ================================================================

    cv::Mat camera_matrix_;

    cv::Mat distortion_coefficients_;

    bool camera_calibrated_;


    // undistort 재사용 맵.
    //
    // cv::undistort() 는 호출할 때마다 내부에서
    // initUndistortRectifyMap 을 다시 만든다 (480x360 float 맵
    // 두 장, 약 1.4 MB). intrinsic 은 CameraInfo 로 한 번 정해지면
    // 바뀌지 않으므로 맵도 한 번만 만들고 remap 만 돌린다.
    cv::Mat undistort_map1_;

    cv::Mat undistort_map2_;

    // 맵을 만든 영상 크기. 크기가 바뀌면 다시 만든다.
    cv::Size undistort_map_size_{0, 0};
};

#endif  // KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_