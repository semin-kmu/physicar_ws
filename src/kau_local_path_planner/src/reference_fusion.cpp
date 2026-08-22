#include "kau_local_path_planner/reference_fusion.hpp"

namespace kau
{
namespace local_path_planner
{

ReferenceFusion::ReferenceFusion(
    const Curve & global_path, std::string ref_mode,
    double w_lane, double lane_gate)
: global_path_(global_path), ref_mode_(std::move(ref_mode)),
  w_lane_(w_lane), lane_gate_(lane_gate)
{
}

void ReferenceFusion::project(const Point2 & p, const Curve * lane_curve)
{
    state_ = global_path_.nearest(p, state_);
    s0_ = state_.s;

    if (ref_mode_ == "lane" && lane_curve != nullptr)
    {
        lane_backbone_ = true;
        eval_state_ = lane_curve->nearestGlobal(p);
        lane_used_ = true;
        fuse_ = false;   // backbone 자체가 lane 이므로 종점 융합은 없음
        return;
    }

    lane_backbone_ = false;
    eval_state_ = state_;
    lane_used_ = (ref_mode_ == "fuse" && lane_curve != nullptr && w_lane_ > 0.0);
    // Python: fuse = lane_used and backbone is self.gp (backbone 이 global
    // path 일 때만 fuse -- lane_backbone_ 이 false 인 경우 항상 참)
    fuse_ = lane_used_;
}

Frame ReferenceFusion::evalFrame(
    double s, const Curve * lane_curve, float lane_confidence) const
{
    const Curve & backbone = lane_backbone_ ? *lane_curve : global_path_;
    return refFrame(backbone, s, lane_curve, lane_confidence, fuse_);
}

Frame ReferenceFusion::refFrame(
    const Curve & backbone, double s, const Curve * lane_curve,
    float lane_confidence, bool fuse) const
{
    Frame f = referenceFrame(backbone, backbone.wrapS(s));
    if (!fuse || lane_curve == nullptr)
    {
        return f;
    }

    const TrackState st = lane_curve->nearestGlobal(f.point);
    if (st.dist > lane_gate_ || !(st.u > 1e-6 && st.u < 1.0 - 1e-6))
    {
        return f;   // lane 미관측 구간 -> 외삽 금지
    }

    const auto & c = lane_curve->seg(st.seg);
    const Point2 q = kau::bezier::evalSeg(c, st.u);
    const double w = w_lane_ * static_cast<double>(lane_confidence);

    Frame out;
    out.point = f.point + w * (q - f.point);
    out.heading = f.heading + w * kau::control::wrapPi(
        kau::bezier::heading(c, st.u) - f.heading);
    out.kappa = f.kappa + w * (kau::bezier::curvature(c, st.u) - f.kappa);
    return out;
}

}  // namespace local_path_planner
}  // namespace kau
