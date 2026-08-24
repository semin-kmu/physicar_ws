#include "kau_gui/map_view.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QWheelEvent>

#include "kau_gui/theme.hpp"


namespace kau_gui
{

namespace
{

// 표시물이 이 시간 넘게 갱신되지 않으면 흐리게 그린다. 낡은 데이터를
// 최신인 척 그리는 것이 디버깅 도구에서 가장 위험하다.
constexpr double STALE_S = 1.0;

constexpr double SCAN_STALE_S = 1.0;

constexpr double MIN_SCALE = 5.0;      // px/m

constexpr double MAX_SCALE = 4000.0;

}  // namespace


MapView::MapView(QWidget * parent)
: QWidget(parent)
{
    setAutoFillBackground(false);

    setMouseTracking(false);

    setFocusPolicy(Qt::NoFocus);      // 단축키는 MainWindow 가 받는다
}


void MapView::setVehicleSize(double length_m, double width_m)
{
    veh_len_ = length_m;

    veh_wid_ = width_m;
}


void MapView::setSnapshot(const Snapshot * s)
{
    snap_ = s;

    // 맵을 처음 받은 프레임에 한 번만 전체가 보이도록 맞춘다.
    // 위젯 크기가 아직 안 잡혔으면 미룬다 (0 폭으로 맞추면 배율이 튄다).
    if (!fitted_ && s != nullptr && s->map.valid && width() > 10 &&
        height() > 10)
    {
        fitted_ = true;

        resetView();
    }
}


void MapView::resetView()
{
    if (snap_ != nullptr && snap_->map.valid)
    {
        const MapImage & m = snap_->map;

        center_ = QPointF(
            m.origin_x + m.widthM() * 0.5, m.origin_y + m.heightM() * 0.5);

        const double sx = width() / std::max(m.widthM(), 1e-6);

        const double sy = height() / std::max(m.heightM(), 1e-6);

        scale_ = std::clamp(std::min(sx, sy) * 0.92, MIN_SCALE, MAX_SCALE);
    }
    else
    {
        center_ = QPointF(0.0, 0.0);

        scale_ = 200.0;
    }

    follow_ = false;

    update();
}


void MapView::setFollow(bool on)
{
    follow_ = on;

    update();
}


QPointF MapView::toScreen(double wx, double wy) const
{
    return QPointF(
        width() * 0.5 + (wx - center_.x()) * scale_,
        height() * 0.5 - (wy - center_.y()) * scale_);
}


bool MapView::stale(const double stamp, double timeout) const
{
    return snap_ == nullptr || (snap_->now - stamp) > timeout;
}


// ====================================================================
// 렌더
// ====================================================================

void MapView::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    p.fillRect(rect(), theme::MAP_BG);

    if (snap_ == nullptr)
    {
        return;
    }

    if (follow_ && snap_->pose.valid)
    {
        center_ = QPointF(snap_->pose.x, snap_->pose.y);
    }

    // 뒤에서 앞으로. 겹칠 때 무엇이 위에 와야 하는지가 곧 우선순위다.
    drawMap(p);

    drawScan(p);

    drawPath(
        p, snap_->path_global, theme::PATH_GLOBAL, 1.5,
        stale(snap_->path_global.stamp, 10.0));

    drawPath(
        p, snap_->path_lane, theme::PATH_LANE, 1.8,
        stale(snap_->path_lane.stamp, STALE_S));

    drawPath(
        p, snap_->path_local, theme::PATH_LOCAL, 2.6,
        stale(snap_->path_local.stamp, STALE_S));

    drawObstacles(p);

    drawVehicle(p);

    drawLookahead(p);

    drawScaleBar(p);

    drawLegend(p);
}


void MapView::drawMap(QPainter & p)
{
    const MapImage & m = snap_->map;

    if (!m.valid || m.img.isNull())
    {
        return;
    }

    // 이미지는 row 0 이 월드 y 최대다 (types.hpp MapImage 규약).
    // 따라서 화면 좌상단 = 월드 (origin_x, origin_y + heightM).
    const QPointF tl = toScreen(m.origin_x, m.origin_y + m.heightM());

    const QPointF br = toScreen(m.origin_x + m.widthM(), m.origin_y);

    // 점유격자는 최근접 보간이 맞다. 스무딩하면 벽 경계가 번져서
    // 실제 장애물 위치를 눈으로 재기 어려워진다.
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);

    p.drawImage(QRectF(tl, br), m.img);
}


void MapView::drawScan(QPainter & p)
{
    const Latest<Polyline> & s = snap_->scan;

    if (!s.got || s.value.empty())
    {
        return;
    }

    const bool old = stale(s.stamp, SCAN_STALE_S);

    p.setPen(QPen(old ? theme::SCAN_STALE : theme::SCAN, 2.0));

    // 점을 하나씩 drawPoint 하면 호출 비용이 지배적이다. 한 번에 넘긴다.
    QPolygonF pts;

    pts.reserve(static_cast<int>(s.value.size()));

    for (const QPointF & q : s.value)
    {
        pts << toScreen(q.x(), q.y());
    }

    p.drawPoints(pts);
}


void MapView::drawPath(
    QPainter & p, const Latest<Polyline> & path, const QColor & c,
    double width_px, bool is_stale)
{
    if (!path.got || path.value.size() < 2)
    {
        return;
    }

    QColor col = c;

    if (is_stale)
    {
        col.setAlpha(70);
    }

    p.setPen(QPen(col, width_px, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    QPolygonF poly;

    poly.reserve(static_cast<int>(path.value.size()));

    for (const QPointF & q : path.value)
    {
        poly << toScreen(q.x(), q.y());
    }

    p.drawPolyline(poly);
}


void MapView::drawObstacles(QPainter & p)
{
    const auto & o = snap_->obstacles;

    if (!o.got)
    {
        return;
    }

    const bool old = stale(o.stamp, STALE_S);

    QColor fill = theme::OBSTACLE;

    fill.setAlpha(old ? 50 : 130);

    p.setPen(QPen(old ? theme::OBSTACLE.darker(160) : theme::OBSTACLE, 1.6));

    p.setBrush(fill);

    for (const Obstacle & ob : o.value)
    {
        const QPointF c = toScreen(ob.x, ob.y);

        const double r = ob.r * scale_;

        p.drawEllipse(c, r, r);
    }

    p.setBrush(Qt::NoBrush);
}


void MapView::drawVehicle(QPainter & p)
{
    const Pose2D & v = snap_->pose;

    // valid=false 는 "TF 를 못 얻음" 또는 "TF 가 낡음" 이다. 좌표가 있으면
    // 마지막 위치를 흐리게 남긴다. 아예 지우면 어디서 멈췄는지 모른다.
    if (!v.valid && (v.x == 0.0 && v.y == 0.0 && v.yaw == 0.0))
    {
        return;
    }

    const double L = veh_len_ * scale_;

    const double W = veh_wid_ * scale_;

    p.save();

    p.translate(toScreen(v.x, v.y));

    // 화면은 y 가 아래로 증가하므로 회전 부호가 월드와 반대다.
    p.rotate(-v.yaw * 180.0 / M_PI);

    QColor body = theme::VEHICLE;

    if (!v.valid)
    {
        body.setAlpha(70);
    }

    p.setPen(QPen(body, 2.0));

    p.setBrush(QColor(body.red(), body.green(), body.blue(), v.valid ? 60 : 25));

    p.drawRect(QRectF(-L * 0.5, -W * 0.5, L, W));

    // 진행 방향 표시. 사각형만 있으면 앞뒤를 구분할 수 없다.
    QPolygonF nose;

    nose << QPointF(L * 0.5, 0.0)
         << QPointF(L * 0.2, -W * 0.35)
         << QPointF(L * 0.2, W * 0.35);

    p.setBrush(body);

    p.drawPolygon(nose);

    p.restore();

    p.setBrush(Qt::NoBrush);
}


void MapView::drawLookahead(QPainter & p)
{
    const auto & la = snap_->lookahead;

    if (!la.got || stale(la.stamp, STALE_S))
    {
        return;
    }

    const QPointF c = toScreen(la.value.x(), la.value.y());

    p.setPen(QPen(theme::LOOKAHEAD, 2.0));

    p.setBrush(Qt::NoBrush);

    p.drawEllipse(c, 5.0, 5.0);

    p.drawLine(QPointF(c.x() - 8, c.y()), QPointF(c.x() + 8, c.y()));

    p.drawLine(QPointF(c.x(), c.y() - 8), QPointF(c.x(), c.y() + 8));

    // 차량과 이어 준다. Ld 가 어디에 놓였는지 한눈에 보인다.
    if (snap_->pose.valid)
    {
        QColor line = theme::LOOKAHEAD;

        line.setAlpha(90);

        p.setPen(QPen(line, 1.0, Qt::DashLine));

        p.drawLine(toScreen(snap_->pose.x, snap_->pose.y), c);
    }
}


// 화면 우하단. 줌을 바꿔도 거리 감각을 잃지 않게 한다.
void MapView::drawScaleBar(QPainter & p)
{
    // 1-2-5 계열에서 화면 폭의 1/5 에 가장 가까운 길이를 고른다.
    const double target_m = (width() / 5.0) / std::max(scale_, 1e-6);

    const double mag = std::pow(10.0, std::floor(std::log10(target_m)));

    const double norm = target_m / mag;

    const double nice = (norm < 1.5) ? 1.0 : (norm < 3.5) ? 2.0
                      : (norm < 7.5) ? 5.0 : 10.0;

    const double len_m = nice * mag;

    const double len_px = len_m * scale_;

    const double x1 = width() - 24.0;

    const double x0 = x1 - len_px;

    const double y = height() - 22.0;

    p.setPen(QPen(theme::TEXT_DIM, 1.5));

    p.drawLine(QPointF(x0, y), QPointF(x1, y));

    p.drawLine(QPointF(x0, y - 4), QPointF(x0, y + 4));

    p.drawLine(QPointF(x1, y - 4), QPointF(x1, y + 4));

    QFont f = p.font();

    f.setPointSizeF(8.5);

    p.setFont(f);

    const QString label = (len_m >= 1.0)
        ? QString("%1 m").arg(len_m, 0, 'g', 3)
        : QString("%1 cm").arg(len_m * 100.0, 0, 'g', 3);

    p.drawText(
        QRectF(x0, y - 20, len_px, 16), Qt::AlignCenter, label);
}


void MapView::drawLegend(QPainter & p)
{
    struct Row
    {
        QColor  color;
        QString label;
        bool    live;
    };

    const auto fresh = [this](const double t, double to)
        {
            return !stale(t, to);
        };

    const std::vector<Row> rows = {
        {theme::PATH_GLOBAL, "global path",
         snap_->path_global.got},
        {theme::PATH_LOCAL, "local path",
         snap_->path_local.got && fresh(snap_->path_local.stamp, STALE_S)},
        {theme::PATH_LANE, "lane center",
         snap_->path_lane.got && fresh(snap_->path_lane.stamp, STALE_S)},
        {theme::SCAN, "scan",
         snap_->scan.got && fresh(snap_->scan.stamp, SCAN_STALE_S)},
        {theme::OBSTACLE, "obstacle",
         snap_->obstacles.got && fresh(snap_->obstacles.stamp, STALE_S)},
        {theme::LOOKAHEAD, "lookahead",
         snap_->lookahead.got && fresh(snap_->lookahead.stamp, STALE_S)},
        {theme::VEHICLE, "vehicle (TF)", snap_->pose.valid},
    };

    QFont f = p.font();

    f.setPointSizeF(9.0);

    p.setFont(f);

    const int pad   = 8;
    const int row_h = 17;
    const int box   = 10;
    const int w     = 132;
    const int h     = pad * 2 + row_h * static_cast<int>(rows.size());

    const int x = width() - w - 12;
    const int y = 12;

    p.setPen(Qt::NoPen);

    p.setBrush(theme::PANEL_BG_TRANSLUCENT);

    p.drawRoundedRect(QRectF(x, y, w, h), 4, 4);

    int cy = y + pad;

    for (const Row & r : rows)
    {
        QColor c = r.color;

        // 안 들어오는 항목은 흐리게. 범례가 곧 "무엇이 살아 있는가" 다.
        if (!r.live)
        {
            c.setAlpha(60);
        }

        p.setPen(Qt::NoPen);

        p.setBrush(c);

        p.drawRect(QRectF(x + pad, cy + (row_h - box) / 2.0, box, box));

        p.setPen(r.live ? theme::TEXT : theme::TEXT_DIM);

        p.drawText(
            QRectF(x + pad + box + 6, cy, w - pad * 2 - box - 6, row_h),
            Qt::AlignVCenter | Qt::AlignLeft, r.label);

        cy += row_h;
    }

    p.setBrush(Qt::NoBrush);
}


// ====================================================================
// 조작
// ====================================================================

void MapView::wheelEvent(QWheelEvent * e)
{
    const double steps = e->angleDelta().y() / 120.0;

    if (steps == 0.0)
    {
        return;
    }

    const QPointF pos = e->position();

    // 커서 밑의 월드 좌표가 제자리에 남도록 center 를 보정한다.
    const double wx =
        center_.x() + (pos.x() - width() * 0.5) / scale_;

    const double wy =
        center_.y() - (pos.y() - height() * 0.5) / scale_;

    const double old = scale_;

    scale_ = std::clamp(scale_ * std::pow(1.15, steps), MIN_SCALE, MAX_SCALE);

    if (scale_ == old)
    {
        return;
    }

    if (!follow_)
    {
        center_ = QPointF(
            wx - (pos.x() - width() * 0.5) / scale_,
            wy + (pos.y() - height() * 0.5) / scale_);
    }

    update();
}


void MapView::mousePressEvent(QMouseEvent * e)
{
    if (e->button() != Qt::LeftButton)
    {
        return;
    }

    dragging_ = true;

    drag_last_ = e->localPos();
}


void MapView::mouseMoveEvent(QMouseEvent * e)
{
    if (!dragging_)
    {
        return;
    }

    const QPointF d = e->localPos() - drag_last_;

    drag_last_ = e->localPos();

    // 팬은 추종과 양립할 수 없다. 손으로 옮기면 추종을 놓는다.
    follow_ = false;

    center_ -= QPointF(d.x() / scale_, -d.y() / scale_);

    update();
}


void MapView::mouseReleaseEvent(QMouseEvent *)
{
    dragging_ = false;
}

}  // namespace kau_gui
