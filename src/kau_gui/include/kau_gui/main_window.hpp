// ====================================================================
// main_window.hpp
//
// 레이아웃
//     +--------------------------------------------------+
//     | StatusBar  (가로 전체, 세로 고정)                  |
//     +---------------------+----------------------------+
//     | PlotStrip x5        |  MapView                   |
//     | (좌, splitter)      |  (우, 범례 우상단)          |
//     +---------------------+----------------------------+
//
// 렌더는 QTimer 로 render_hz (기본 10 Hz). ROS 콜백 도착 주기와 독립이다.
// 플롯 데이터는 콜백에서 전량 쌓이므로 50 Hz 신호도 누락되지 않는다.
//
// 단축키
//     space  화면 갱신 일시정지 (수신은 계속. 순간 관찰용)
//     r      뷰 리셋
//     f      차량 추종 토글
// ====================================================================

#ifndef KAU_GUI__MAIN_WINDOW_HPP_
#define KAU_GUI__MAIN_WINDOW_HPP_

#include <memory>

#include <QMainWindow>

#include "kau_gui/ros_bridge.hpp"
#include "kau_gui/types.hpp"


class QLabel;
class QTimer;


namespace kau_gui
{

class MapView;
class PlotStrip;
class StatusBar;


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<RosBridge> bridge);

protected:
    void keyPressEvent(QKeyEvent * e) override;

private:
    void onTick();

    void buildUi();

    std::shared_ptr<RosBridge> bridge_;

    // 프레임마다 갱신되는 복사본. 위젯들이 이 주소를 들고 있으므로
    // 멤버로 유지해야 한다 (지역 변수로 두면 dangling).
    Snapshot snap_;

    StatusBar * status_ = nullptr;
    MapView   * map_    = nullptr;

    PlotStrip * plot_speed_    = nullptr;
    PlotStrip * plot_steer_    = nullptr;
    PlotStrip * plot_ld_       = nullptr;
    PlotStrip * plot_heading_  = nullptr;
    PlotStrip * plot_cte_      = nullptr;

    QLabel * hint_ = nullptr;

    QTimer * timer_ = nullptr;

    bool paused_ = false;

    double history_s_ = 30.0;
};

}  // namespace kau_gui

#endif  // KAU_GUI__MAIN_WINDOW_HPP_
