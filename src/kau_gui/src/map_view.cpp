#include "kau_gui/map_view.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QFont>
#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QWheelEvent>

#include "kau_gui/plot_axis.hpp"
#include "kau_gui/theme.hpp"


namespace kau_gui
{

namespace
{

// 표시물이 이 시간 넘게 갱신되지 않으면 흐리게 그린다. 낡은 데이터를
// 최신인 척 그리는 것이 디버깅 도구에서 가장 위험하다.
constexpr double STALE_S = 1.0;

constexpr double SCAN_STALE_S = 1.0;

// 전역 경로는 latched 라 한 번만 온다. 다른 것과 같은 잣대로 재면 안 된다.
constexpr double GLOBAL_STALE_S = 1e9;

constexpr double MIN_SCALE = 5.0;      // px/m

constexpr double MAX_SCALE = 4000.0;

constexpr int TOP_PAD   = 8;
constexpr int RIGHT_PAD = 10;


QFont monoFont(double pt)
{
    QFont f("monospace", 9);

    f.setStyleHint(QFont::Monospace);

    f.setPointSizeF(pt);

    return f;
}

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


void MapView::setDisplayUnit(const QString & unit)
{
    unit_name_ = unit;

    unit_mult_ = (unit == "cm") ? 100.0 : 1.0;
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


QRectF MapView::plotRect() const
{
    const double bottom =
        axis::TICK_LEN + axis::X_TICK_H + axis::X_LABEL_H;

    return QRectF(
        axis::LEFT_MARGIN, TOP_PAD,
        std::max(1.0, static_cast<double>(width() - axis::LEFT_MARGIN - RIGHT_PAD)),
        std::max(1.0, height() - TOP_PAD - bottom));
}


void MapView::resetView()
{
    const QRectF plot = plotRect();

    if (snap_ != nullptr && snap_->map.valid)
    {
        const MapImage & m = snap_->map;

        center_ = QPointF(
            m.origin_x + m.widthM() * 0.5, m.origin_y + m.heightM() * 0.5);

        const double sx = plot.width() / std::max(m.widthM(), 1e-6);

        const double sy = plot.height() / std::max(m.heightM(), 1e-6);

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
    const QRectF plot = plotRect();

    return QPointF(
        plot.center().x() + (wx - center_.x()) * scale_,
        plot.center().y() - (wy - center_.y()) * scale_);
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

    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), theme::PANEL_BG);

    const QRectF plot = plotRect();

    if (snap_ == nullptr)
    {
        axis::drawFrame(p, plot);

        return;
    }

    if (follow_ && snap_->pose.valid)
    {
        center_ = QPointF(snap_->pose.x, snap_->pose.y);
    }

    // 축 · 격자는 표시물 아래에 깔린다.
    drawAxes(p, plot);

    // plot 밖으로 삐져나가지 않게 자른다. 축 눈금 자리를 침범하면
    // 숫자를 못 읽는다.
    p.save();

    p.setClipRect(plot);

    // 뒤에서 앞으로. 겹칠 때 무엇이 위에 와야 하는지가 곧 우선순위다.
    drawMap(p, plot);

    drawScan(p);

    drawPath(
        p, snap_->path_global, theme::PATH_GLOBAL, 2.0,
        stale(snap_->path_global.stamp, GLOBAL_STALE_S));

    drawPath(
        p, snap_->path_lane, theme::PATH_LANE, 2.4,
        stale(snap_->path_lane.stamp, STALE_S));

    drawPath(
        p, snap_->path_local, theme::PATH_LOCAL, 3.0,
        stale(snap_->path_local.stamp, STALE_S));

    drawObstacles(p);

    drawVehicle(p);

    drawLookahead(p);

    p.restore();

    axis::drawFrame(p, plot);

    drawHud(p, plot);

    drawLegend(p, plot);
}


void MapView::drawAxes(QPainter & p, const QRectF & plot)
{
    // 화면에 보이는 월드 범위를 표시 단위로 환산해 눈금을 뽑는다.
    const double half_w = plot.width() * 0.5 / scale_;

    const double half_h = plot.height() * 0.5 / scale_;

    const double x0 = (center_.x() - half_w) * unit_mult_;

    const double x1 = (center_.x() + half_w) * unit_mult_;

    const double y0 = (center_.y() - half_h) * unit_mult_;

    const double y1 = (center_.y() + half_h) * unit_mult_;

    const std::vector<double> xt = axis::niceTicks(x0, x1, 6);

    const std::vector<double> yt = axis::niceTicks(y0, y1, 5);

    // viz.map_plot 의 showGrid(alpha=0.25)
    axis::drawGrid(p, plot, xt, x0, x1, yt, y0, y1, 64);

    axis::drawYAxis(
        p, plot, yt, y0, y1, QString("y [%1]").arg(unit_name_));

    axis::drawXAxis(
        p, plot, xt, x0, x1, QString("x [%1]").arg(unit_name_));
}


void MapView::drawMap(QPainter & p, const QRectF &)
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

    // 맵 경계선 (viz.draw_map_border)
    p.setPen(QPen(theme::MAP_BORDER, 1.0));

    p.setBrush(Qt::NoBrush);

    p.drawRect(QRectF(tl, br));

    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
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
        col.setAlpha(60);
    }

    p.setPen(QPen(col, width_px, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    p.setBrush(Qt::NoBrush);

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

    // viz.disk 와 같은 규약: 테두리는 진하게, 채움은 alpha 90
    QColor fill = theme::OBSTACLE;

    fill.setAlpha(old ? 35 : 90);

    QColor edge = theme::OBSTACLE;

    if (old)
    {
        edge.setAlpha(70);
    }

    p.setPen(QPen(edge, 1.0));

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

    p.setBrush(QColor(body.red(), body.green(), body.blue(), v.valid ? 50 : 20));

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

        line.setAlpha(110);

        p.setPen(QPen(line, 1.0, Qt::DashLine));

        p.drawLine(toScreen(snap_->pose.x, snap_->pose.y), c);
    }
}


// ====================================================================
// HUD · 범례
// ====================================================================

// 맵에 그려지는 것들의 수신 상태만 담는다. 제어 수치는 좌측 플롯 소관이다.
// "지금 화면에 무엇이 살아서 그려지고 있는가" 가 이 상자의 전부다.
void MapView::drawHud(QPainter & p, const QRectF & plot)
{
    struct Row
    {
        QString label;
        QString value;
        bool    ok;
    };

    const auto age = [this](double stamp)
        {
            return snap_->now - stamp;
        };

    const auto pathRow =
        [&](const char * name, const Latest<Polyline> & l, double timeout)
        {
            if (!l.got)
            {
                return Row{name, "미수신", false};
            }

            const bool ok = age(l.stamp) <= timeout;

            return Row{
                name,
                QString("%1 pt  %2s")
                    .arg(static_cast<int>(l.value.size()), 4)
                    .arg(age(l.stamp), 5, 'f', 1),
                ok};
        };

    const auto tfRow = [](const char * name, const TfLink & t)
        {
            if (!t.got)
            {
                return Row{name, "미수신", false};
            }

            return Row{
                name, QString("%1s").arg(t.age, 5, 'f', 1), t.ok};
        };

    std::vector<Row> rows;

    rows.push_back(
        pathRow("global path", snap_->path_global, GLOBAL_STALE_S));

    rows.push_back(pathRow("local path ", snap_->path_local, STALE_S));

    rows.push_back(pathRow("lane center", snap_->path_lane, STALE_S));

    rows.push_back(tfRow("tf map>odom", snap_->tf_map_odom));

    rows.push_back(tfRow("tf odom>base", snap_->tf_odom_base));

    if (!snap_->obstacles.got)
    {
        rows.push_back(Row{"obstacles  ", "미수신", false});
    }
    else
    {
        const bool ok = age(snap_->obstacles.stamp) <= STALE_S;

        rows.push_back(
            Row{"obstacles  ",
                QString("%1 EA  %2s")
                    .arg(static_cast<int>(snap_->obstacles.value.size()), 4)
                    .arg(age(snap_->obstacles.stamp), 5, 'f', 1),
                ok});
    }


    p.setFont(monoFont(8.5));

    const QFontMetricsF fm(p.font());

    double label_w = 0.0;

    double value_w = 0.0;

    for (const Row & r : rows)
    {
        label_w = std::max(label_w, fm.horizontalAdvance(r.label));

        value_w = std::max(value_w, fm.horizontalAdvance(r.value));
    }

    const double pad = 7.0;

    const double dot = 7.0;

    const double row_h = fm.height() + 2.0;

    const double w = pad * 2 + dot + 6.0 + label_w + 10.0 + value_w;

    const double h = pad * 2 + row_h * static_cast<double>(rows.size());

    const QRectF box(plot.left() + 8, plot.top() + 8, w, h);

    // viz.hud 와 같은 규약: 흰 반투명 채움 + 회색 테두리
    p.setPen(QPen(theme::BORDER, 1.0));

    p.setBrush(theme::PANEL_BG_TRANSLUCENT);

    p.drawRect(box);

    double y = box.top() + pad;

    for (const Row & r : rows)
    {
        p.setPen(Qt::NoPen);

        p.setBrush(r.ok ? theme::OK : theme::FAULT);

        p.drawEllipse(
            QPointF(box.left() + pad + dot * 0.5, y + row_h * 0.5),
            dot * 0.5, dot * 0.5);

        p.setPen(r.ok ? theme::TEXT : theme::TEXT_DIM);

        p.drawText(
            QRectF(box.left() + pad + dot + 6.0, y, label_w, row_h),
            Qt::AlignVCenter | Qt::AlignLeft, r.label);

        p.drawText(
            QRectF(
                box.left() + pad + dot + 6.0 + label_w + 10.0, y,
                value_w, row_h),
            Qt::AlignVCenter | Qt::AlignRight, r.value);

        y += row_h;
    }

    p.setBrush(Qt::NoBrush);
}


// viz.py 의 addLegend(offset=(-10, 10)) 와 같은 자리. 선 견본 + 이름.
void MapView::drawLegend(QPainter & p, const QRectF & plot)
{
    struct Row
    {
        QColor  color;
        QString label;
        bool    live;
    };

    const auto fresh = [this](double t, double to)
        {
            return !stale(t, to);
        };

    const std::vector<Row> rows = {
        {theme::PATH_GLOBAL, "global path", snap_->path_global.got},
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
        {theme::VEHICLE, "vehicle", snap_->pose.valid},
    };

    // drawHud 가 painter 폰트를 monospace 로 바꿔 놓았다. 범례는 본문
    // 폰트가 맞으므로 위젯 기본값에서 다시 잡는다.
    QFont f = font();

    f.setPointSizeF(8.5);

    p.setFont(f);

    const QFontMetricsF fm(p.font());

    double text_w = 0.0;

    for (const Row & r : rows)
    {
        text_w = std::max(text_w, fm.horizontalAdvance(r.label));
    }

    const double pad = 7.0;

    const double sample = 18.0;      // 선 견본 길이

    const double row_h = fm.height() + 2.0;

    const double w = pad * 2 + sample + 6.0 + text_w;

    const double h = pad * 2 + row_h * static_cast<double>(rows.size());

    const QRectF box(plot.right() - w - 10, plot.top() + 8, w, h);

    p.setPen(QPen(theme::BORDER, 1.0));

    p.setBrush(theme::PANEL_BG_TRANSLUCENT);

    p.drawRect(box);

    double y = box.top() + pad;

    for (const Row & r : rows)
    {
        QColor c = r.color;

        // 안 들어오는 항목은 흐리게. 범례가 곧 "무엇이 살아 있는가" 다.
        if (!r.live)
        {
            c.setAlpha(55);
        }

        p.setPen(QPen(c, 2.4));

        p.drawLine(
            QPointF(box.left() + pad, y + row_h * 0.5),
            QPointF(box.left() + pad + sample, y + row_h * 0.5));

        p.setPen(r.live ? theme::TEXT : theme::TEXT_DIM);

        p.drawText(
            QRectF(box.left() + pad + sample + 6.0, y, text_w, row_h),
            Qt::AlignVCenter | Qt::AlignLeft, r.label);

        y += row_h;
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

    const QRectF plot = plotRect();

    // 커서 밑의 월드 좌표가 제자리에 남도록 center 를 보정한다.
    const double wx = center_.x() + (pos.x() - plot.center().x()) / scale_;

    const double wy = center_.y() - (pos.y() - plot.center().y()) / scale_;

    const double old = scale_;

    scale_ = std::clamp(scale_ * std::pow(1.15, steps), MIN_SCALE, MAX_SCALE);

    if (scale_ == old)
    {
        return;
    }

    if (!follow_)
    {
        center_ = QPointF(
            wx - (pos.x() - plot.center().x()) / scale_,
            wy + (pos.y() - plot.center().y()) / scale_);
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
