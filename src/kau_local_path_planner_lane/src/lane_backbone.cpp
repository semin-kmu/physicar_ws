#include "kau_local_path_planner_lane/lane_backbone.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "kau_local_path_planner_lane/bezier_ext.hpp"
#include "kau_local_path_planner_lane/candidate_generator.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::bezier::Ctrl;
using kau::bezier::Point2;

Curve extendCurve(const Curve & cv, double target_length, int bound_depth)
{
    const double real_length = cv.length();
    if (real_length >= target_length || cv.nseg() == 0)
    {
        return cv;
    }
    const Frame f = referenceFrame(cv, real_length);
    const Knot start{f.point, f.heading, f.kappa};

    const double remain = target_length - real_length;
    const double dth = start.kappa * remain;
    const double mid_th = start.theta + 0.5 * dth;
    const Point2 tail_p{
        start.p.x + remain * std::cos(mid_th),
        start.p.y + remain * std::sin(mid_th)};
    const Knot tail{tail_p, kau::control::wrapPi(start.theta + dth), start.kappa};

    const auto [seg, kb] = fitSegment(start, tail, bound_depth);
    (void)kb;
    if (!seg)
    {
        return cv;
    }
    std::vector<Ctrl> segs(cv.ctrl().begin(), cv.ctrl().end());
    segs.push_back(*seg);
    return Curve(std::move(segs), cv.closed());
}

BackboneResult buildBackbone(
    const Curve * left, const Curve * right, const Curve * center,
    double kappa0, double target_length, int bound_depth)
{
    BackboneResult out;
    if (center == nullptr || center->nseg() == 0)
    {
        return out;
    }
    const double length = center->length();
    const Knot ego{Point2{0.0, 0.0}, 0.0, kappa0};

    // bridge: ego -> center 위 한 점. 몇 개 후보 fraction 중 kappa_bound 가
    // 가장 낮은 것을 고른다 -- ego(heading 고정 0) 에서 center 시작점까지
    // chord 가 짧으면 관측 heading 이 이미 돌아있어 quintic Hermite 가
    // kappa_lim 을 넘는 과대 곡률을 요구하는 걸 KAU_AMET_Test 세션에서
    // 실측 확인했다 (짧은 chord 일수록 심함).
    std::optional<Ctrl> best_seg;
    double best_kb = std::numeric_limits<double>::infinity();
    double best_s = length;
    for (double frac : {1.0 / 3.0, 0.5, 2.0 / 3.0, 1.0})
    {
        const double s = frac * length;
        const Frame f = referenceFrame(*center, s);
        const Knot k{f.point, f.heading, f.kappa};
        const auto [seg, kb] = fitSegment(ego, k, bound_depth);
        if (seg && kb < best_kb)
        {
            best_seg = seg;
            best_kb = kb;
            best_s = s;
        }
    }
    if (!best_seg)
    {
        return out;
    }

    // bridge 이후는 center 를 재적합 없이(split 만으로) 그대로 이어붙인다
    // -- 고정 fraction 재적합은 center 가 짧을 때(한쪽 edge 만 검출 등)
    // 조각당 chord 가 너무 짧아져 곡률이 튀는 걸 실측으로 확인했다.
    int seg_idx = 0;
    double u = 1.0;
    center->locate(best_s, seg_idx, u);
    std::vector<Ctrl> segs;
    segs.push_back(*best_seg);
    if (u < 1.0 - 1e-9)
    {
        Ctrl unused_left, tail_seg;
        kau::bezier::split(center->seg(seg_idx), u, unused_left, tail_seg);
        segs.push_back(tail_seg);
    }
    for (int j = seg_idx + 1; j < center->nseg(); ++j)
    {
        segs.push_back(center->seg(j));
    }

    Curve backbone(std::move(segs), false);
    out.backbone = extendCurve(backbone, target_length, bound_depth);
    if (left != nullptr && left->nseg() > 0)
    {
        out.left = extendCurve(*left, target_length, bound_depth);
    }
    if (right != nullptr && right->nseg() > 0)
    {
        out.right = extendCurve(*right, target_length, bound_depth);
    }
    return out;
}

}  // namespace local_path_planner_lane
}  // namespace kau
