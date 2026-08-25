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

// 세그먼트 하나가 담당할 최대 회전각. quintic Hermite 1 개로 표현 가능한
// 원호 한계에서 온다 -- 이보다 크게 잡으면 sigma 후보(kSigmaRatios) 를 어떻게
// 골라도 중간이 부풀어 곡률이 튄다.
constexpr double kMaxTurnPerSegRad = M_PI / 4.0;   // 45deg

Curve extendCurve(const Curve & cv, double target_length, int bound_depth)
{
    const double real_length = cv.length();
    if (real_length >= target_length || cv.nseg() == 0)
    {
        return cv;
    }
    const Frame f = referenceFrame(cv, real_length);
    Knot cur{f.point, f.heading, f.kappa};

    const double remain = target_length - real_length;

    // 2026-08-25 (곡선 구간 미추종 수정): 예전 코드는 tail 점을 시작점에서
    // **호길이** remain 만큼 떨어진 곳에 놓았다. 등곡률 원호의 현(chord)
    // 길이는 호길이가 아니라 2*sin(dth/2)/kappa 이고, 둘의 비는
    // 1/sinc(dth/2) 라 회전각이 커질수록 벌어진다:
    //     dth  17deg -> +0.4%   57deg -> +4.3%   126deg -> +23%
    // 코너(R=60cm, remain=132cm -> dth=126deg)에서 tail 이 25cm 밖에 찍히고,
    // 그걸 quintic 하나로 억지로 이으면서 backbone 곡률이 0.01839 ->
    // 0.02255 로 부풀어 차량 한계(0.02022)를 넘겼다. 그 순간 전 후보가
    // kappa_bound 로 탈락하고 leastViolation 이 R=24cm 짜리 경로를 발행한다
    // (차량 최소회전반경 49.5cm -- 물리적으로 못 따라간다).
    // 실측(합성 원호): backbone 기하오차 R=60 에서 22.6cm -> 0.1cm.
    //
    // 더해서, 회전각이 아무리 커도 세그먼트 1 개로 붙이던 것을
    // kMaxTurnPerSegRad 단위로 쪼갠다.
    const double dth_total = cur.kappa * remain;
    const int n = std::max(
        1, static_cast<int>(std::ceil(std::abs(dth_total) / kMaxTurnPerSegRad)));
    const double step = remain / n;

    std::vector<Ctrl> segs(cv.ctrl().begin(), cv.ctrl().end());
    for (int i = 0; i < n; ++i)
    {
        const double dth = cur.kappa * step;
        // |dth| 가 아주 작으면 2*sin(dth/2)/kappa 가 0/0 이라 직선(step)으로.
        const double chord = (std::abs(dth) < 1e-9)
            ? step
            : 2.0 * std::sin(0.5 * dth) / cur.kappa;
        const double mid_th = cur.theta + 0.5 * dth;
        const Knot next{
            Point2{cur.p.x + chord * std::cos(mid_th),
                   cur.p.y + chord * std::sin(mid_th)},
            kau::control::wrapPi(cur.theta + dth), cur.kappa};

        const auto [seg, kb] = fitSegment(cur, next, bound_depth);
        (void)kb;
        if (!seg)
        {
            break;   // 여기까지만 늘린다 (원본 실패 시 cv 반환과 같은 방침)
        }
        segs.push_back(*seg);
        cur = next;
    }
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
