// ====================================================================
// arbiter.hpp -- 경로 소스 중재 (ROS 의존성 없음)
//
// 장애물이 없으면 차선(/lane/center)을 그대로 따라가고, 있을 때만 회피
// 경로(/path/local)로 넘어간다.
//
// 왜 이렇게 나눴나: 실제 트랙 폴리곤 위 기구학 시뮬(1 바퀴 28.3m,
// v=0.5m/s, 조향 서보 300deg/s) 실측이다.
//
//   방식                          최대 |cte|   최소 노면여유   이탈 시간
//   /lane/center 직접 추종(PP)       10.1cm      +10.8cm        0%
//   곡률 FF + PP                     20.1cm       +4.7cm        0%
//   곡률 검출시 조향 max(bang-bang)  >300cm       -9.5cm       14.0%
//
// 장애물이 없는 구간에서는 플래너 기하를 주행 루프에서 빼는 쪽이 실측으로
// 더 정확하다. 플래너는 그것이 실제로 필요한 순간(장애물)에만 쓴다.
//
// 이 헤더는 순수 로직이라 노드 없이 단위 테스트가 된다.
// ====================================================================

#ifndef KAU_PATH_ARBITER__ARBITER_HPP_
#define KAU_PATH_ARBITER__ARBITER_HPP_

namespace kau
{
namespace path_arbiter
{

enum class Mode
{
    kLane,    // 장애물 없음 -> /lane/center
    kAvoid,   // 장애물 있음 -> /path/local
};

enum class Source
{
    kNone,    // 고른 소스가 끊겼다 -- 아무것도 발행하지 않는다 (제어기가 선다)
    kLane,
    kLocal,
};

struct ArbiterParams
{
    // --- 장애물 관심 영역 (자차 기준, cm) ---
    // 이 상자 안의 장애물만 회피 대상으로 본다. 뒤나 옆으로 한참 떨어진
    // 것 때문에 회피 모드로 넘어가면 평상시 주행이 플래너를 타게 된다.
    double trigger_min_x_cm      = 0.0;
    double trigger_max_x_cm      = 150.0;
    double trigger_half_width_cm = 40.0;

    // --- 히스테리시스 ---
    // 비대칭이다. 진입은 즉시(안전), 해제는 느리게(떨림 방지).
    //
    // release_sec 은 "장애물을 마지막으로 본 뒤 회피 모드를 유지할 시간"
    // 이다. v=0.5m/s 에서 2.0s = 100cm -- 장애물을 지나쳐 뒤로 보낼 만큼.
    // 짧으면 장애물이 인지 경계에서 깜빡일 때 5Hz/14Hz 두 소스를 오가며
    // 조향이 튄다.
    double engage_sec  = 0.0;
    double release_sec = 2.0;

    // 이 시간 넘게 새 메시지가 없으면 그 소스는 끊긴 것으로 본다.
    double lane_timeout_sec  = 0.5;   // 14Hz -> 최대 7 프레임
    double local_timeout_sec = 1.0;   //  5Hz -> 5 주기

    // 회피 중에 /path/local 이 끊기면 차선으로 내려갈 것인가.
    //
    // 기본 false 다. 장애물이 있는데 장애물을 모르는 경로로 내려가는 건
    // 장애물을 향해 그대로 가는 것과 같다. 아무것도 발행하지 않으면
    // 제어기가 timeout 으로 선다 -- 그쪽이 안전하다.
    bool fallback_to_lane_on_local_loss = false;
};

struct Decision
{
    Mode   mode   = Mode::kLane;
    Source source = Source::kNone;
    // 진단용. 왜 이 결정이 나왔는지 (로그/GUI 에 그대로 찍는다).
    const char * reason = "init";
};

// 장애물 하나가 회피를 유발할 만한 위치인가. x/y/r 은 자차 기준 cm.
// r 을 반폭에 더해 실제 점유 폭으로 본다.
bool obstacleRelevant(double x_cm, double y_cm, double r_cm,
                      const ArbiterParams & p);

class Arbiter
{
public:
    explicit Arbiter(ArbiterParams params);

    // 한 관측 주기의 결과를 반영한다.
    //
    // valid=false 는 **"장애물이 없다"가 아니라 "모른다"** 다
    // (ObstacleCircleArray 계약: non-OK 프레임은 항상 빈 배열이며, 그건
    // 인식 결과 사용 불가라는 뜻이지 노면이 비었다는 뜻이 아니다).
    // 모르는 동안에는 해제 타이머를 진행시키지 않아 현재 모드를 유지한다.
    void updateObstacles(bool valid, bool relevant, double now_sec);

    // lane_fresh/local_fresh 는 노드가 수신 시각으로 판정해 넘긴다.
    Decision decide(bool lane_fresh, bool local_fresh, double now_sec);

    Mode mode() const { return mode_; }

private:
    ArbiterParams params_;
    Mode   mode_ = Mode::kLane;
    // 마지막으로 "관심 영역 안 장애물" 을 본 시각. 없으면 음수.
    double last_relevant_sec_ = -1.0;
    // 처음으로 연속 검출이 시작된 시각 (engage_sec 디바운스용).
    double relevant_since_sec_ = -1.0;
    // 마지막으로 유효한(status OK) 관측을 받은 시각.
    double last_valid_sec_ = -1.0;
};

}  // namespace path_arbiter
}  // namespace kau

#endif  // KAU_PATH_ARBITER__ARBITER_HPP_
