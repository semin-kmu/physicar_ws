#include "kau_path_arbiter/arbiter.hpp"

#include <cmath>

namespace kau
{
namespace path_arbiter
{

bool obstacleRelevant(double x_cm, double y_cm, double r_cm,
                      const ArbiterParams & p)
{
    if (!std::isfinite(x_cm) || !std::isfinite(y_cm) || !std::isfinite(r_cm))
    {
        return false;
    }
    const double r = r_cm > 0.0 ? r_cm : 0.0;
    // 종방향은 원의 앞/뒤 끝으로 판정한다 -- 중심만 보면 상자 경계에
    // 걸친 큰 장애물을 놓친다.
    if (x_cm + r < p.trigger_min_x_cm || x_cm - r > p.trigger_max_x_cm)
    {
        return false;
    }
    return std::abs(y_cm) - r <= p.trigger_half_width_cm;
}

Arbiter::Arbiter(ArbiterParams params)
: params_(params)
{
}

void Arbiter::updateObstacles(bool valid, bool relevant, double now_sec)
{
    if (!valid)
    {
        // 모르는 상태. 타이머를 건드리지 않아 현재 모드가 그대로 유지된다.
        return;
    }
    last_valid_sec_ = now_sec;
    if (relevant)
    {
        last_relevant_sec_ = now_sec;
        if (relevant_since_sec_ < 0.0)
        {
            relevant_since_sec_ = now_sec;
        }
    }
    else
    {
        relevant_since_sec_ = -1.0;
    }
}

Decision Arbiter::decide(bool lane_fresh, bool local_fresh, double now_sec)
{
    // ---- 1. 모드 갱신 (히스테리시스) ----
    if (mode_ == Mode::kLane)
    {
        // 진입: engage_sec 동안 연속 검출되어야 한다 (기본 0 = 즉시).
        const bool engaged =
            relevant_since_sec_ >= 0.0 &&
            (now_sec - relevant_since_sec_) >= params_.engage_sec;
        if (engaged)
        {
            mode_ = Mode::kAvoid;
        }
    }
    else
    {
        // 해제: 마지막 검출로부터 release_sec 이 지나야 한다.
        // last_relevant_sec_ 은 "모르는 동안" 갱신되지 않지만, 그때는
        // now_sec 만 흘러 해제가 앞당겨질 수 있다. 그래서 유효 관측
        // 시각(last_valid_sec_) 기준으로 잰다 -- 인지가 죽어 있는 동안
        // 시간이 흘렀다는 이유만으로 회피를 그만두지 않게 한다.
        const double clear_for =
            (last_valid_sec_ >= 0.0 && last_relevant_sec_ >= 0.0)
                ? (last_valid_sec_ - last_relevant_sec_)
                : -1.0;
        if (clear_for >= params_.release_sec)
        {
            mode_ = Mode::kLane;
            last_relevant_sec_ = -1.0;
            relevant_since_sec_ = -1.0;
        }
    }

    // ---- 2. 그 모드에서 실제로 낼 소스 ----
    Decision d;
    d.mode = mode_;
    if (mode_ == Mode::kLane)
    {
        if (lane_fresh)
        {
            d.source = Source::kLane;
            d.reason = "lane";
        }
        else
        {
            d.source = Source::kNone;
            d.reason = "lane_stale";
        }
        return d;
    }

    if (local_fresh)
    {
        d.source = Source::kLocal;
        d.reason = "avoid";
        return d;
    }
    if (params_.fallback_to_lane_on_local_loss && lane_fresh)
    {
        // 기본으로는 여기 안 온다 (헤더 주석 참고). 장애물을 모르는 경로로
        // 내려가는 것이라, 명시적으로 켠 경우에만 쓴다.
        d.source = Source::kLane;
        d.reason = "avoid_fallback_lane";
        return d;
    }
    d.source = Source::kNone;
    d.reason = "local_stale";
    return d;
}

}  // namespace path_arbiter
}  // namespace kau
