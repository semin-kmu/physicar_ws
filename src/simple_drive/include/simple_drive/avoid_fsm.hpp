// ====================================================================
// avoid_fsm.hpp
//
// 장애물 회피 상태머신. 하드코딩 타이머 시퀀스다.
//
//   IDLE --트리거--> DODGE(반대쪽 전타, dodge_s)
//                     -> COUNTER(같은쪽 전타, counter_s)
//                     -> HOLD(0 도 직진, hold_s, 재트리거 금지)
//                     -> IDLE (차선추종 복귀)
//
// --------------------------------------------------------------
// 왜 "5 도 1 초" 가 아닌가 -- 기하가 허락하지 않는다
// --------------------------------------------------------------
// 조향각 delta 로 거리 d 를 가는 동안의 횡변위는
//
//     y = R(1 - cos(d/R)) ~= d^2 / (2R),   R = wheelbase / tan(delta)
//
// 이다. 속도가 아니라 **주행거리**가 정한다 -- 천천히 가면 같은 시간에
// 덜 움직인다. 5 도면 R = 0.18/tan5 = 2.06 m 라, v=0.3 m/s 로 1 초
// (d=0.30 m) 를 꺾어도 횡변위가
//
//     0.30^2 / (2 x 2.06) = 0.022 m = 2.2 cm
//
// 밖에 안 된다. 콘 반경 0.09 + 차 반폭 0.1025 = 0.19 m 를 비켜야 하는데
// 2 cm 는 아무것도 아니다. 5 도는 회피가 아니라 노이즈다.
//
// 전타(20 도, R=0.4945)라도 0.19 m 를 벌려면 d = sqrt(2R*0.19) = 0.43 m
// 를 가야 한다. 이건 속도를 낮춰도 줄지 않는다.
//
// --------------------------------------------------------------
// S 자 (DODGE + COUNTER) 의 기하
// --------------------------------------------------------------
// 같은 R 로 같은 호길이 L 씩 좌우로 꺾으면(theta = L/R):
//
//     총 횡변위   = 2R(1 - cos theta)
//     총 종방향   = 2R sin theta
//     최종 방위   = 0            <- 방위가 원래대로 돌아온다
//
// 목표 횡변위 0.20 m -> 1-cos theta = 0.202 -> theta = 0.640 rad (36.6 도)
//   L = R*theta = 0.316 m,  v=0.30 m/s -> 1.05 s   (dodge_s = counter_s)
//   종방향 = 2*0.4945*sin(0.640) = 0.59 m
//
// 즉 "전타 1.05 초 + 반대 전타 1.05 초" 로 59 cm 가는 동안 20 cm 옆으로
// 평행이동한다. 차로 반폭이 0.317 - 0.1025 = 0.21 m 라 딱 들어간다.
//
// --------------------------------------------------------------
// 왜 HOLD 가 필요한가
// --------------------------------------------------------------
// S 자가 끝난 순간 차는 중앙선에서 20 cm 옆에 있다. 여기서 곧바로
// 차선추종(밴뱅)을 풀면 밴뱅은 중앙선으로 "돌아가려" 하고, 그 방향에
// 바로 그 장애물이 있다. HOLD 구간에서 0 도로 직진해 장애물을 지나친
// 뒤에 풀어야 한다. HOLD 는 재트리거 금지 구간이기도 하다 -- 같은
// 장애물이 아직 시야에 있어서 곧바로 또 트리거되기 때문이다.
// ====================================================================

#ifndef SIMPLE_DRIVE__AVOID_FSM_HPP_
#define SIMPLE_DRIVE__AVOID_FSM_HPP_

#include <cmath>

#include <backup_msgs/msg/obstacle_circle_array.hpp>

namespace simple_drive
{

enum class AvoidState
{
  Idle,
  Dodge,
  Counter,
  Hold,
};

inline const char * toString(AvoidState s)
{
  switch (s) {
    case AvoidState::Dodge: return "DODGE";
    case AvoidState::Counter: return "COUNTER";
    case AvoidState::Hold: return "HOLD";
    default: return "IDLE";
  }
}

class AvoidFsm
{
public:
  struct Params
  {
    // 트리거 게이트. base_link, 원점 후륜축, x 전방 / y 좌측 +.
    //
    // x_max 는 S 자 종방향(0.59 m)보다 커야 한다. 그보다 가까이서
    // 시작하면 옆으로 다 비키기 전에 장애물에 닿는다.
    double trigger_x_min_m = 0.15;
    double trigger_x_max_m = 0.90;
    // 콘 반경 0.09 + 차 반폭 0.1025 = 0.19. 그보다 옆이면 안 부딪힌다.
    double trigger_y_abs_m = 0.20;

    double dodge_deg = 20.0;   // 전타. 5 도로는 못 비킨다 (위 주석)
    double dodge_s = 1.05;     // v_avoid 0.30 기준. 속도 바꾸면 같이 바꾼다
    double counter_s = 1.05;   // dodge_s 와 같아야 방위가 0 으로 돌아온다
    double hold_s = 1.00;      // 0 도 직진으로 장애물을 지나치는 구간
  };

  // 기본 인자로 Params() 를 쓸 수 없는 이유는 bang_bang.hpp 와 같다.
  AvoidFsm() = default;

  explicit AvoidFsm(const Params & p)
  : p_(p) {}

  void setParams(const Params & p) {p_ = p;}

  AvoidState state() const {return state_;}

  // 회피가 조향을 쥐고 있는가. HOLD 도 포함이다 -- HOLD 의 0 도는
  // "차선추종이 우연히 0 도" 가 아니라 "직진하라" 는 지시다.
  bool ownsSteer() const {return state_ != AvoidState::Idle;}

  double triggerY() const {return trigger_y_;}

  void reset()
  {
    state_ = AvoidState::Idle;
    timer_s_ = 0.0;
    dodge_sign_ = 0;
    trigger_y_ = 0.0;
  }

  // 반환값: 회피가 지시하는 조향각 [deg]. ownsSteer() 가 false 면 무의미.
  double update(const backup_msgs::msg::ObstacleCircleArray * obs, double dt)
  {
    timer_s_ += dt;

    switch (state_) {
      case AvoidState::Idle: {
          double y = 0.0;
          if (obs && pickTarget(*obs, y)) {
          // 장애물이 왼쪽(y>0)이면 오른쪽(-)으로 피한다.
          // y 가 정확히 0 이면 (정면) 오른쪽으로 보낸다 -- 어느 쪽이든
          // 되지만 결정이 매 tick 흔들리지 않게 부호를 고정한다.
            dodge_sign_ = (y > 0.0) ? -1 : 1;
            trigger_y_ = y;
            state_ = AvoidState::Dodge;
            timer_s_ = 0.0;
          }
          break;
        }
      case AvoidState::Dodge:
        if (timer_s_ >= p_.dodge_s) {
          state_ = AvoidState::Counter;
          timer_s_ = 0.0;
        }
        break;
      case AvoidState::Counter:
        if (timer_s_ >= p_.counter_s) {
          state_ = AvoidState::Hold;
          timer_s_ = 0.0;
        }
        break;
      case AvoidState::Hold:
        if (timer_s_ >= p_.hold_s) {
          state_ = AvoidState::Idle;
          timer_s_ = 0.0;
          dodge_sign_ = 0;
        }
        break;
    }

    if (state_ == AvoidState::Dodge) {return dodge_sign_ * p_.dodge_deg;}
    if (state_ == AvoidState::Counter) {return -dodge_sign_ * p_.dodge_deg;}
    return 0.0;  // Hold / Idle
  }

private:
  // 진로 안에서 가장 가까운 장애물. 없으면 false.
  bool pickTarget(const backup_msgs::msg::ObstacleCircleArray & obs, double & y_out) const
  {
    // status 가 OK 가 아니면 배열이 비어 있다는 것이 발행측 계약이지만,
    // 계약을 믿고 분기를 생략하지 않는다.
    if (obs.status != backup_msgs::msg::ObstacleCircleArray::STATUS_OK) {return false;}

    bool found = false;
    double best_x = 0.0;
    for (const auto & o : obs.obstacles) {
      if (o.center_x < p_.trigger_x_min_m || o.center_x > p_.trigger_x_max_m) {continue;}
      if (std::fabs(o.center_y) > p_.trigger_y_abs_m) {continue;}
      if (!found || o.center_x < best_x) {
        found = true;
        best_x = o.center_x;
        y_out = o.center_y;
      }
    }
    return found;
  }

  Params p_;
  AvoidState state_ = AvoidState::Idle;
  double timer_s_ = 0.0;
  int dodge_sign_ = 0;
  double trigger_y_ = 0.0;
};

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__AVOID_FSM_HPP_
