#include "kau_gui/plot_axis.hpp"

#include <algorithm>
#include <cmath>

#include <QFont>
#include <QPainter>
#include <QPen>

#include "kau_gui/theme.hpp"


namespace kau_gui
{
namespace axis
{

namespace
{

// 축 글자는 본문보다 작게. viz.py 의 8pt 눈금과 같은 인상.
QFont tickFont(const QPainter & p)
{
    QFont f = p.font();

    f.setPointSizeF(8.0);

    f.setBold(false);

    return f;
}

}  // namespace


std::vector<double> niceTicks(double lo, double hi, int want)
{
    std::vector<double> out;

    if (!std::isfinite(lo) || !std::isfinite(hi) || hi <= lo || want < 2)
    {
        return out;
    }

    const double raw = (hi - lo) / static_cast<double>(want);

    if (raw <= 0.0)
    {
        return out;
    }

    const double mag = std::pow(10.0, std::floor(std::log10(raw)));

    const double n = raw / mag;

    // 1-2-5-10 중 raw 를 넘지 않으면서 가장 가까운 것
    const double mult = (n < 1.5) ? 1.0 : (n < 3.0) ? 2.0 : (n < 7.0) ? 5.0 : 10.0;

    const double step = mult * mag;

    if (step <= 0.0)
    {
        return out;
    }

    const double first = std::ceil(lo / step) * step;

    // 부동소수 누적을 피하려고 index 로 센다.
    for (int i = 0; i < 200; ++i)
    {
        const double v = first + step * i;

        if (v > hi + step * 1e-9)
        {
            break;
        }

        // -0.0 이 "-0" 으로 찍히는 것을 막는다.
        out.push_back(std::abs(v) < step * 1e-9 ? 0.0 : v);
    }

    return out;
}


double stepOf(const std::vector<double> & ticks)
{
    return ticks.size() < 2 ? 0.0 : (ticks[1] - ticks[0]);
}


QString tickLabel(double v, double step)
{
    int dec = 0;

    if (step > 0.0 && step < 1.0)
    {
        dec = std::min(6, static_cast<int>(std::ceil(-std::log10(step))));
    }

    return QString::number(v, 'f', dec);
}


void drawGrid(
    QPainter & p, const QRectF & plot,
    const std::vector<double> & xt, double x0, double x1,
    const std::vector<double> & yt, double y0, double y1,
    int alpha)
{
    QColor g = theme::GRID;

    g.setAlpha(alpha);

    p.setPen(QPen(g, 1.0));

    const double xspan = x1 - x0;

    const double yspan = y1 - y0;

    if (xspan > 0.0)
    {
        for (const double v : xt)
        {
            const double x = plot.left() + (v - x0) / xspan * plot.width();

            p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
        }
    }

    if (yspan > 0.0)
    {
        for (const double v : yt)
        {
            const double y = plot.bottom() - (v - y0) / yspan * plot.height();

            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
        }
    }
}


void drawFrame(QPainter & p, const QRectF & plot)
{
    p.setPen(QPen(theme::BORDER, 1.0));

    p.setBrush(Qt::NoBrush);

    p.drawRect(plot);
}


void drawYAxis(
    QPainter & p, const QRectF & plot, const std::vector<double> & yt,
    double y0, double y1, const QString & name)
{
    const double span = y1 - y0;

    if (span <= 0.0)
    {
        return;
    }

    p.setFont(tickFont(p));

    const double step = stepOf(yt);

    p.setPen(theme::TEXT);

    for (const double v : yt)
    {
        const double y = plot.bottom() - (v - y0) / span * plot.height();

        // plot 밖으로 반쯤 걸치는 눈금은 글자가 잘리므로 건너뛴다.
        if (y < plot.top() - 1 || y > plot.bottom() + 1)
        {
            continue;
        }

        p.setPen(QPen(theme::AXIS, 1.0));

        p.drawLine(
            QPointF(plot.left() - TICK_LEN, y), QPointF(plot.left(), y));

        p.setPen(theme::TEXT);

        p.drawText(
            QRectF(
                plot.left() - Y_TICK_W, y - 7.0,
                Y_TICK_W - TICK_LEN - 2.0, 14.0),
            Qt::AlignRight | Qt::AlignVCenter, tickLabel(v, step));
    }

    if (name.isEmpty())
    {
        return;
    }

    // y 축 이름은 세로로 눕힌다 (viz.py setLabel("left", ...) 와 같은 배치).
    p.save();

    p.translate(plot.left() - Y_TICK_W - 2.0, plot.center().y());

    p.rotate(-90.0);

    p.setPen(theme::TEXT);

    p.drawText(
        QRectF(-plot.height() * 0.5, -Y_LABEL_W, plot.height(), Y_LABEL_W),
        Qt::AlignCenter, name);

    p.restore();
}


void drawXAxis(
    QPainter & p, const QRectF & plot, const std::vector<double> & xt,
    double x0, double x1, const QString & name)
{
    const double span = x1 - x0;

    if (span <= 0.0)
    {
        return;
    }

    p.setFont(tickFont(p));

    const double step = stepOf(xt);

    for (const double v : xt)
    {
        const double x = plot.left() + (v - x0) / span * plot.width();

        if (x < plot.left() - 1 || x > plot.right() + 1)
        {
            continue;
        }

        p.setPen(QPen(theme::AXIS, 1.0));

        p.drawLine(
            QPointF(x, plot.bottom()), QPointF(x, plot.bottom() + TICK_LEN));

        p.setPen(theme::TEXT);

        p.drawText(
            QRectF(x - 30.0, plot.bottom() + TICK_LEN, 60.0, X_TICK_H),
            Qt::AlignHCenter | Qt::AlignTop, tickLabel(v, step));
    }

    if (name.isEmpty())
    {
        return;
    }

    p.setPen(theme::TEXT);

    p.drawText(
        QRectF(
            plot.left(), plot.bottom() + TICK_LEN + X_TICK_H,
            plot.width(), X_LABEL_H),
        Qt::AlignHCenter | Qt::AlignTop, name);
}

}  // namespace axis
}  // namespace kau_gui
