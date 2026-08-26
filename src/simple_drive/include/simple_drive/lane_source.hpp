// ====================================================================
// lane_source.hpp
//
// 어느 차선 근거로 횡오차를 낼 것인가. 헤더 온리, 상태 없음.
//
//   1) 중앙선(/backup/lane/path) 이 있으면 그것
//   2) 없으면 좌/우 흰선을 lane_width_m 만큼 안쪽으로 밀어 복원
//   3) 둘 다 없으면 invalid
//
// 흰선 -> 중앙선 복원의 부호
//   좌측 + 규약이다. 좌 흰선은 중앙선보다 왼쪽(y 가 크다)이므로
//   중앙선 = 좌흰선 - w. 우 흰선은 오른쪽이므로 중앙선 = 우흰선 + w.
//   둘 다 보이면 평균이 곧 회랑 중심이라 한쪽만 쓸 때보다 강건하다.
//
// 흰선은 노란선을 놓친 순간에도 살아남는다는 것이 실측이다
// (회피 전타 직후 path 0 Hz / left,right 14 Hz). 노란 중앙선은 점선이라
// 검출기가 "REJECT THIN" 게이트로 경로 발행을 자주 막는데(출발선에서도
// 그렇다) 그때도 흰선은 나온다.
//
// 다만 흰선을 그대로 믿으면 안 된다. 실측(2026-08-26):
//
//     t=1.6  L-R = 0.610 m   복원 불일치 0.024   정상
//     t=2.0  L-R = 0.415 m   복원 불일치 0.219   도로 폭이 41 cm?
//     t=2.4  L-R = 0.063 m   복원 불일치 0.570   붕괴
//
// 도로 폭은 0.634 m 로 고정인데 주행 중 좌우가 저렇게 어긋난다. 저
// 0.570 짜리 샘플이 실제로 엉뚱한 방향의 전타를 만들었다. 좌우가 서로
// 안 맞으면 **어느 쪽이 틀렸는지 알 방법이 없으므로** 평균도 한쪽 선택도
// 하지 않고 흰선을 통째로 버린다. 근거 없이 달리는 것보다 근거 없음을
// 인정하고 유예로 넘기는 편이 안전하다.
//
// 너무 짧은 흰선도 버린다 -- 20 cm 짜리 조각에서 뽑은 방위는 잡음이다.
// ====================================================================

#ifndef SIMPLE_DRIVE__LANE_SOURCE_HPP_
#define SIMPLE_DRIVE__LANE_SOURCE_HPP_

#include <cmath>

#include <backup_msgs/msg/backup_path.hpp>

#include "simple_drive/path_sample.hpp"

namespace simple_drive
{

enum class LaneOrigin
{
  None,
  Center,
  WhiteBoth,
  WhiteLeft,
  WhiteRight,
};

inline const char * toString(LaneOrigin o)
{
  switch (o) {
    case LaneOrigin::Center: return "center";
    case LaneOrigin::WhiteBoth: return "white_both";
    case LaneOrigin::WhiteLeft: return "white_left";
    case LaneOrigin::WhiteRight: return "white_right";
    default: return "none";
  }
}

struct LaneEstimate
{
  bool valid = false;
  LaneOrigin origin = LaneOrigin::None;
  double lateral = 0.0;  // [m] lookahead 지점의 중앙선 횡오차. 좌측 +
  double heading = 0.0;  // [rad]
  double s = 0.0;        // [m] 실제로 쓴 호길이
};

class LaneResolver
{
public:
  struct Params
  {
    double lookahead_m = 0.28;
    double valid_length_margin_m = 0.05;
    int samples_per_seg = 32;
    // 중앙선 중심 <-> 흰선 중심 [m]. lane_detector.yaml 의
    // lane_width_cm 31.7 과 같은 값이어야 한다.
    double lane_width_m = 0.317;
    // 흰선을 근거로 채택할 최소 검출 길이 [m].
    double white_min_valid_length_m = 0.15;
    // 좌/우 흰선에서 각각 복원한 중앙선이 이 이상 어긋나면 그 쌍은
    // 신뢰할 수 없다. 도로 폭은 고정이라 정상이면 수 mm 안에 든다.
    double pair_max_disagree_m = 0.10;
  };

  explicit LaneResolver(const Params & p)
  : p_(p) {}

  LaneResolver() = default;

  void setParams(const Params & p) {p_ = p;}
  const Params & params() const {return p_;}

  // center / left / right 는 각각 신선하지 않으면 nullptr 로 준다.
  LaneEstimate resolve(
    const backup_msgs::msg::BackupPath * center,
    const backup_msgs::msg::BackupPath * left,
    const backup_msgs::msg::BackupPath * right) const
  {
    LaneEstimate out;

    if (center) {
      const Lookahead la = sample(*center);
      if (la.valid) {
        out.valid = true;
        out.origin = LaneOrigin::Center;
        out.lateral = la.lateral;
        out.heading = la.heading;
        out.s = la.s;
        return out;
      }
    }

    // 흰선 복원. 중앙선이 없을 때만 온다.
    bool have_l = false;
    bool have_r = false;
    Lookahead la_l;
    Lookahead la_r;

    if (left && left->valid_length >= p_.white_min_valid_length_m) {
      la_l = sample(*left);
      have_l = la_l.valid;
    }
    if (right && right->valid_length >= p_.white_min_valid_length_m) {
      la_r = sample(*right);
      have_r = la_r.valid;
    }

    if (have_l && have_r) {
      out.valid = true;
      out.origin = LaneOrigin::WhiteBoth;
      out.lateral = 0.5 * ((la_l.lateral - p_.lane_width_m) +
        (la_r.lateral + p_.lane_width_m));
      out.heading = 0.5 * (la_l.heading + la_r.heading);
      out.s = 0.5 * (la_l.s + la_r.s);
    } else if (have_l) {
      out.valid = true;
      out.origin = LaneOrigin::WhiteLeft;
      out.lateral = la_l.lateral - p_.lane_width_m;
      out.heading = la_l.heading;
      out.s = la_l.s;
    } else if (have_r) {
      out.valid = true;
      out.origin = LaneOrigin::WhiteRight;
      out.lateral = la_r.lateral + p_.lane_width_m;
      out.heading = la_r.heading;
      out.s = la_r.s;
    }

    return out;
  }

private:
  Lookahead sample(const backup_msgs::msg::BackupPath & p) const
  {
    return lookaheadAt(p, p_.lookahead_m, p_.valid_length_margin_m, p_.samples_per_seg);
  }

  Params p_;
};

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__LANE_SOURCE_HPP_
