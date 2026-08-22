// ====================================================================
// reference_fusion.hpp
//
// 차량 위치를 reference(global path)에 투영하고, lane detection 관측과
// 종점 상태 융합(문서 6.7)한다.
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner/test/planner.py 의
//       LocalPlanner._project / _ref_frame / reference_frame
//
// KAU_AMET_Test 세션 실측: w_lane=1.0 이 heading jitter/candidate switching
// 의 원인일 것으로 의심했으나, w_lane 을 0~1.0 스윕해도 degraded 빈도/
// 충돌률에 측정 가능한 영향이 없었다 (원인 아님, 로직은 그대로 유지).
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__REFERENCE_FUSION_HPP_
#define KAU_LOCAL_PATH_PLANNER__REFERENCE_FUSION_HPP_

#include <string>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner/bezier_ext.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::control::Curve;
using kau::control::TrackState;

class ReferenceFusion
{
public:
    ReferenceFusion(
        const Curve & global_path, std::string ref_mode,
        double w_lane, double lane_gate);

    // global_path_ 를 참조로 들고 있어 복사/이동하면 댕글링된다.
    ReferenceFusion(const ReferenceFusion &) = delete;
    ReferenceFusion & operator=(const ReferenceFusion &) = delete;
    ReferenceFusion(ReferenceFusion &&) = delete;
    ReferenceFusion & operator=(ReferenceFusion &&) = delete;

    // Python: _project. lane_curve 는 이번 사이클 lane 관측(없으면 nullptr).
    // 내부에 backbone/fuse 결정을 캐시해 이후 evalFrame() 호출에 쓴다.
    void project(const Point2 & p, const Curve * lane_curve);

    double s0() const { return s0_; }
    double projectedS() const { return eval_state_.s; }
    bool laneUsed() const { return lane_used_; }

    // Python: reference_frame(s, lane_result) 공개 API (평가/시각화용).
    // 이번 사이클 project() 가 정한 backbone/fuse 결정을 그대로 쓴다.
    Frame evalFrame(
        double s, const Curve * lane_curve, float lane_confidence) const;

private:
    // Python: _ref_frame(backbone, s, lane_result, fuse).
    Frame refFrame(
        const Curve & backbone, double s, const Curve * lane_curve,
        float lane_confidence, bool fuse) const;

    const Curve & global_path_;
    std::string  ref_mode_;
    double       w_lane_;
    double       lane_gate_;

    TrackState state_;         // Python: self._state (global path window 추적)
    double     s0_ = 0.0;      // Python: self._s0

    bool         lane_backbone_ = false;
    bool         lane_used_     = false;
    bool         fuse_          = false;
    TrackState   eval_state_;
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__REFERENCE_FUSION_HPP_
