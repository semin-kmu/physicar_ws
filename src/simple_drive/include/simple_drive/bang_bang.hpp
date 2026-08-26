// ====================================================================
// bang_bang.hpp
//
// 히스테리시스 + 최소유지시간 밴뱅. 출력은 -1 / 0 / +1 세 값뿐이다
// (부호는 조향 방향, 좌측 +).
//
// 왜 히스테리시스인가:
//   임계 하나로 부호만 뒤집으면 e 가 임계 근처에 있을 때 매 tick
//   상태가 바뀐다. 50 Hz 면 25 Hz 채터고, 조향 서보는 그걸 못 따라가
//   결과적으로 "평균 0 도 + 소음" 이 된다. 진입 임계를 이탈 임계보다
//   크게 두면 한 번 꺾은 뒤 중앙 근처까지 돌아와야 놓는다.
//
// 왜 최소유지시간인가:
//   히스테리시스만으로는 e 가 임계를 크게 넘나드는 코너에서 여전히
//   빠른 스위칭이 난다. 하드웨어 조향속도가 600 deg/s 라 0->20 도에
//   33 ms 가 걸린다. 그보다 짧게 지시를 뒤집으면 서보가 목표에 닿기
//   전에 명령이 바뀌어 실제 조향각이 지시와 무관해진다.
// ====================================================================

#ifndef SIMPLE_DRIVE__BANG_BANG_HPP_
#define SIMPLE_DRIVE__BANG_BANG_HPP_

#include <cmath>

namespace simple_drive
{

class BangBang
{
public:
  struct Params
  {
    double enter_m = 0.05;   // |e| 가 이 값을 넘으면 전타 진입
    double exit_m = 0.02;    // |e| 가 이 값 아래로 내려오면 0 도 복귀
    double min_hold_s = 0.06;  // 상태 전환 후 최소 유지 시간
  };

  // 기본 인자로 Params() 를 쓰면 안 된다 -- 중첩 struct 의 기본
  // 멤버 초기화자가 바깥 클래스 정의가 끝나기 전에 필요해져
  // C++17 에서 컴파일이 안 된다. 생성자를 둘로 나눈다.
  BangBang() = default;

  explicit BangBang(const Params & p)
  : p_(p) {}

  void setParams(const Params & p) {p_ = p;}

  int state() const {return state_;}

  void reset()
  {
    state_ = 0;
    // 유지시간을 "이미 다 채운" 상태로 둔다. 리셋 직후 첫 전환까지
    // min_hold_s 를 또 기다리면 경로 복구 / 회피 종료 직후 몇 tick 을
    // 직진으로 흘려보낸다.
    held_s_ = kFree;
  }

  // e: lookahead 지점의 횡오차 [m]. 좌측 +.
  //    e > 0 이면 경로가 왼쪽에 있다 -> 좌조향(+1).
  // dt: 이번 tick 간격 [s]
  int update(double e, double dt)
  {
    held_s_ += dt;

    if (!std::isfinite(e)) {
      // 근거가 없으면 직진이 전타보다 안전하다.
      // (backup_common::clampAbs 의 NaN 처리와 같은 판단)
      state_ = 0;
      held_s_ = kFree;
      return state_;
    }

    const int desired = desiredState(e);
    if (desired != state_ && held_s_ >= p_.min_hold_s) {
      state_ = desired;
      held_s_ = 0.0;
    }
    return state_;
  }

private:
  int desiredState(double e) const
  {
    const double a = std::fabs(e);
    if (state_ == 0) {
      if (a >= p_.enter_m) {return (e > 0.0) ? 1 : -1;}
      return 0;
    }
    // 전타 중. 부호가 뒤집힐 만큼 반대로 벗어났으면 바로 반대 전타.
    if (a >= p_.enter_m) {return (e > 0.0) ? 1 : -1;}
    if (a <= p_.exit_m) {return 0;}
    return state_;  // 두 임계 사이 = 유지
  }

  // "유지시간 제약이 없다" 를 뜻하는 값. dt 를 더해도 커지기만 한다.
  static constexpr double kFree = 1e9;

  Params p_;
  int state_ = 0;
  double held_s_ = kFree;
};

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__BANG_BANG_HPP_
