// ====================================================================
// lane_types.hpp
//
// 파이프라인 단계 사이를 오가는 자료형. 노드 클래스에서 분리해
// 두어야 기하/경로 헬퍼가 클래스 밖에서도 같은 타입을 쓴다.
// ====================================================================

#ifndef KAU_LANE_DETECTION__LANE_TYPES_HPP_
#define KAU_LANE_DETECTION__LANE_TYPES_HPP_

#include <cstddef>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kau_lane_detection/bezier.hpp"

namespace kau_lane
{

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


enum class PanSearchState
{
    Idle,       // pan = 0. 소실 감시만 한다
    Searching,  // 소실된 쪽으로 전진하며 재검출을 노린다
    Holding,    // 재검출됨. 직선 구간이 나올 때까지 그 각도를 유지
    Returning   // 0 으로 복귀 중
};


struct VehiclePose
{
    bool valid = false;

    double x_m = 0.0;

    double y_m = 0.0;

    double yaw_rad = 0.0;
};


struct KappaTrack
{
    std::size_t seg   = 0;      // 직전 해가 있던 조각
    double      u     = 0.0;    // 그 조각 안의 매개변수
    bool        valid = false;  // false 면 전역 탐색
    int         fails = 0;      // GATE 연속 이탈 횟수
};


struct PanSample
{
    rclcpp::Time stamp;

    double deg;
};


// 한 프레임이 파이프라인 단계를 지나며 채워지는 작업 상태.
// 단계 사이의 유일한 통로다 — 단계 함수는 이것 말고 서로를 모른다.
struct FrameContext
{
    cv::Mat undistorted;
    cv::Mat bev;

    cv::Mat yellow_mask;
    cv::Mat white_mask;

    bool want_debug = false;
    cv::Mat debug_image;

    // 히스토그램 시작점 진단
    double yellow_band = 0.0;
    int yellow_base = -1;
    int left_base = -1;
    int right_base = -1;

    LaneDetectionResult left;
    LaneDetectionResult yellow;
    LaneDetectionResult right;

    CenterFix center_fix;

    // 영상 시각의 pan 각과 그 보정량
    bool pan_angle_clamped = false;
    double camera_yaw_deg = 0.0;
    double pan_angle_fix_deg = 0.0;

    LanePath path;
};

}  // namespace kau_lane

#endif  // KAU_LANE_DETECTION__LANE_TYPES_HPP_
