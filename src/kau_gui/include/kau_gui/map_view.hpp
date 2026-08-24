// ====================================================================
// map_view.hpp
//
// 우측 map 패널. QPainter 2D 로 직접 그린다.
//
// rviz2 와 달리 3D 씬그래프 · 셰이더 · Ogre 를 쓰지 않는다. 필요한 것이
// 탑뷰 2D 뿐이라 3D 파이프라인이 통째로 불필요하기 때문이고, 이것이
// 이 GUI 가 rviz2 보다 가벼운 유일한 이유다.
//
// 좌표: 월드(map 프레임, m) -> 위젯 픽셀.
//     sx = w/2 + (wx - cx) * scale
//     sy = h/2 - (wy - cy) * scale      y 를 뒤집는다 (월드는 y 위쪽)
//
// 조작: 휠 = 줌 / 드래그 = 팬 / f = 차량 추종 / r = 뷰 리셋
// ====================================================================

#ifndef KAU_GUI__MAP_VIEW_HPP_
#define KAU_GUI__MAP_VIEW_HPP_

#include <QColor>
#include <QPointF>
#include <QWidget>

#include "kau_gui/types.hpp"


class QPainter;


namespace kau_gui
{

class MapView : public QWidget
{
    Q_OBJECT

public:
    explicit MapView(QWidget * parent = nullptr);

    void setVehicleSize(double length_m, double width_m);

    // 렌더 타이머가 프레임마다 호출한다. 복사는 MainWindow 가 이미 했다.
    void setSnapshot(const Snapshot * s);

    // 팬/줌을 초기화하고 맵 전체가 보이도록 맞춘다.
    void resetView();

    // 차량을 화면 중앙에 물고 따라간다.
    void setFollow(bool on);

    bool follow() const
    {
        return follow_;
    }

protected:
    void paintEvent(QPaintEvent * e) override;
    void wheelEvent(QWheelEvent * e) override;
    void mousePressEvent(QMouseEvent * e) override;
    void mouseMoveEvent(QMouseEvent * e) override;
    void mouseReleaseEvent(QMouseEvent * e) override;

private:
    QPointF toScreen(double wx, double wy) const;

    void drawMap(QPainter & p);
    void drawScan(QPainter & p);
    void drawPath(
        QPainter & p, const Latest<Polyline> & path, const QColor & c,
        double width_px, bool is_stale);
    void drawObstacles(QPainter & p);
    void drawVehicle(QPainter & p);
    void drawLookahead(QPainter & p);
    void drawLegend(QPainter & p);
    void drawScaleBar(QPainter & p);

    bool stale(const double stamp, double timeout) const;

    const Snapshot * snap_ = nullptr;

    // 화면 중앙에 오는 월드 좌표 [m] 와 배율 [px/m]
    QPointF center_{0.0, 0.0};

    double scale_ = 200.0;

    bool fitted_ = false;      // 맵을 처음 받았을 때 한 번 자동 맞춤

    bool follow_ = false;

    bool dragging_ = false;

    QPointF drag_last_;

    double veh_len_ = 0.30;

    double veh_wid_ = 0.20;
};

}  // namespace kau_gui

#endif  // KAU_GUI__MAP_VIEW_HPP_
