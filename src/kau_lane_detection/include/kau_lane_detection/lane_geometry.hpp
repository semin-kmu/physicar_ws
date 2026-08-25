// ====================================================================
// lane_geometry.hpp
//
// 상태 없는 기하 유틸. 폴리라인 횡거리, 가중 다항 적합, 방향 판정.
// 노드 상태를 하나도 안 쓰므로 클래스 밖 자유 함수로 둔다.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#ifndef KAU_LANE_DETECTION__LANE_GEOMETRY_HPP_
#define KAU_LANE_DETECTION__LANE_GEOMETRY_HPP_

#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

namespace kau_lane
{
namespace geom
{

// 파라미터(int 배열) -> cv::Scalar.
cv::Scalar toScalar(
    const std::vector<int64_t> & v);


// 가중 최소제곱 다항식 적합 v = f(t). Vandermonde + DECOMP_QR.
// weights 가 비어 있으면 균등 가중.
bool polyFitW(
    const std::vector<double> & ts,
    const std::vector<double> & vs,
    const std::vector<double> & weights,
    int order,
    std::vector<double> & coeffs);


// Horner 평가.
double polyEval(
    const std::vector<double> & coeffs,
    double y);


// 폴리라인 위로 투영. 호길이 s 와 거리 d 를 돌려준다.
// 양 끝 구간에서는 바깥으로 외삽을 허용한다 (s < 0 가능).
void projectOnPolyline(
    const std::vector<cv::Point2d> & poly,
    const std::vector<double> & cum,
    const cv::Point2d & p,
    double & s_out,
    double & d_out);


// pts 각 점의 ref 기준 부호 있는 횡거리 [px]. 차량 오른쪽이 +.
std::vector<double> lateralOffsets(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts);


// 위 값의 중앙값. 차선이 통째로 어느 쪽에 있는지 판정용.
double medianLateralOffset(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts);


// 기준선 대비 부호 있는 횡거리 하나. 차량 오른쪽이 +.
//
// lateralOffsets 는 투영 t 를 [0,1] 로 자르지만, 자차 위치는
// 추적 시작점보다 뒤(BEV 아래쪽)에 있어서 그대로 쓰면 첫 점
// 까지의 직선거리가 나와 값이 부풀려진다. 양 끝 구간에서는
// 외삽을 허용한다.
double lateralOffsetAt(
    const std::vector<cv::Point2d> & ref,
    const cv::Point2d & p);


// ref 기준 회랑 [lo, hi] * lane_width 를 벗어나는 순간 잘라낸다.
// 추적이 체커보드 연석이나 다른 구간 차선으로 갈아탄 지점에서
// 끊는다. 통째로 버리지 않고 정상이던 앞부분은 남긴다.
std::vector<cv::Point2d> truncateAtCorridor(
    const std::vector<cv::Point2d> & track,
    const std::vector<cv::Point2d> & ref,
    double want_sign,
    double lo_px,
    double hi_px);


// 순서 있는 점열을 국소 법선(차량 오른쪽) 방향으로 평행이동.
// 90도 코너에서는 x 방향 offset 이 틀리므로 반드시 법선이어야 한다.
std::vector<cv::Point2d> offsetTrack(
    const std::vector<cv::Point2d> & pts,
    double offset_px);


// 순서 있는 점열이 얼마나 휘었는가 [deg]. 판정 불가면 음수.
//
// 앞 절반의 현(chord)과 뒤 절반의 현이 이루는 각이다.
//
// BEV 좌표로 재도 된다. 호모그래피는 직선을 직선으로 보내므로
// "휘었나 안 휘었나" 는 카메라가 돌아가 BEV 가정이 깨진
// 상태에서도 그대로 읽힌다. 다만 각도의 크기는 보존되지
// 않으므로(원근에 따라 늘거나 준다) 이 값은 세상의 도(度)가
// 아니라 BEV 영상 위의 도다. 임계값은 실측으로 잡을 것.
double trackBendDeg(
    const std::vector<cv::Point2d> & pts);

}  // namespace geom
}  // namespace kau_lane

#endif  // KAU_LANE_DETECTION__LANE_GEOMETRY_HPP_
