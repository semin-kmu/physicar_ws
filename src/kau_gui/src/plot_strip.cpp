#include "kau_gui/plot_strip.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <QFont>
#include <QPainter>
#include <QPolygonF>

#include "kau_gui/plot_axis.hpp"
#include "kau_gui/theme.hpp"


namespace kau_gui
{

namespace
{

constexpr int TOP_PAD    = 16;   // 현재값 읽기 줄
constexpr int RIGHT_PAD  = 10;
constexpr int BASE_PAD   = 6;    // x 축이 없는 플롯의 아래 여백

// 마지막 표본이 이보다 오래됐으면 "값 없음" 으로 본다.
constexpr double VALUE_STALE_S = 1.0;


QString fmtValue(double v, int digits)
{
    return QString::number(v, 'f', digits);
}

}  // namespace


PlotStrip::PlotStrip(const QString & y_label, QWidget * parent)
: QWidget(parent),
  y_label_(y_label)
{
    setMinimumHeight(84);

    setAutoFillBackground(false);
}


void PlotStrip::setSeriesNames(const QString & a_name, const QString & b_name)
{
    a_name_ = a_name;

    b_name_ = b_name;
}


void PlotStrip::setMinSpan(double span)
{
    min_span_ = span;
}


void PlotStrip::setIncludeZero(bool on)
{
    include_zero_ = on;
}


void PlotStrip::setShowXAxis(bool on)
{
    show_x_axis_ = on;

    // x 축이 붙는 플롯은 눈금 · 축 이름 자리만큼 더 높아야 그림 영역이
    // 다른 플롯과 같아진다.
    setMinimumHeight(
        on ? 84 + axis::X_TICK_H + axis::X_LABEL_H : 84);
}


void PlotStrip::setData(
    const Series * a, const Series * b, double now, double window_s)
{
    a_ = a;

    b_ = b;

    now_ = now;

    window_ = window_s;
}


void PlotStrip::setNotice(const QString & text)
{
    notice_ = text;
}


// y 축 범위. 창 안의 표본만 본다 (창 밖 값이 축을 잡아끌면 안 된다).
void PlotStrip::computeRange(double * lo, double * hi) const
{
    double mn = std::numeric_limits<double>::infinity();

    double mx = -std::numeric_limits<double>::infinity();

    const double t0 = now_ - window_;

    for (const Series * s : {a_, b_})
    {
        if (s == nullptr)
        {
            continue;
        }

        for (const Sample & smp : s->data())
        {
            if (smp.t < t0)
            {
                continue;
            }

            mn = std::min(mn, smp.v);

            mx = std::max(mx, smp.v);
        }
    }

    if (!std::isfinite(mn) || !std::isfinite(mx))
    {
        mn = -min_span_ * 0.5;

        mx = min_span_ * 0.5;
    }

    if (include_zero_)
    {
        mn = std::min(mn, 0.0);

        mx = std::max(mx, 0.0);
    }

    const double span = std::max(mx - mn, min_span_);

    const double mid = 0.5 * (mn + mx);

    *lo = mid - span * 0.58;

    *hi = mid + span * 0.58;
}


void PlotStrip::drawSeries(
    QPainter & p, const Series * s, const QColor & c, const QRectF & plot,
    double lo, double hi) const
{
    if (s == nullptr || s->empty())
    {
        return;
    }

    const double t0 = now_ - window_;

    const double span = std::max(hi - lo, 1e-9);

    const int w = static_cast<int>(plot.width());

    if (w <= 1)
    {
        return;
    }

    const auto toY = [&](double v)
        {
            return plot.bottom() - (v - lo) / span * plot.height();
        };

    // 픽셀 열마다 min/max. 열에 표본이 하나뿐이면 min==max 라 점이 된다.
    std::vector<double> col_min(
        static_cast<std::size_t>(w), std::numeric_limits<double>::infinity());

    std::vector<double> col_max(
        static_cast<std::size_t>(w), -std::numeric_limits<double>::infinity());

    for (const Sample & smp : s->data())
    {
        if (smp.t < t0)
        {
            continue;
        }

        int col = static_cast<int>((smp.t - t0) / window_ * (w - 1));

        col = std::clamp(col, 0, w - 1);

        const std::size_t i = static_cast<std::size_t>(col);

        col_min[i] = std::min(col_min[i], smp.v);

        col_max[i] = std::max(col_max[i], smp.v);
    }

    p.setPen(QPen(c, 1.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    // 연속 구간마다 폴리라인을 끊는다. 값이 없는 열(발행 중단)에서
    // 선을 이어 버리면 끊긴 사실이 화면에서 사라진다.
    QPolygonF run;

    for (int col = 0; col < w; ++col)
    {
        const std::size_t i = static_cast<std::size_t>(col);

        if (!std::isfinite(col_min[i]))
        {
            if (run.size() >= 2)
            {
                p.drawPolyline(run);
            }
            else if (run.size() == 1)
            {
                p.drawPoint(run.first());
            }

            run.clear();

            continue;
        }

        const double x = plot.left() + col;

        const double y_lo = toY(col_min[i]);

        const double y_hi = toY(col_max[i]);

        // 열 안의 진폭을 세로선으로 남긴다 = 스파이크 보존
        if (std::abs(y_lo - y_hi) > 1.0)
        {
            p.drawLine(QPointF(x, y_lo), QPointF(x, y_hi));
        }

        run << QPointF(x, 0.5 * (y_lo + y_hi));
    }

    if (run.size() >= 2)
    {
        p.drawPolyline(run);
    }
    else if (run.size() == 1)
    {
        p.drawPoint(run.first());
    }
}


// 플롯 위쪽 줄에 계열 이름 + 현재값. 그래프를 읽지 않아도 숫자가 보여야
// 한다. 글자 색을 선 색과 맞춰 어느 숫자가 어느 선인지 즉시 읽히게 한다.
void PlotStrip::drawReadout(QPainter & p, const QRectF & plot) const
{
    QFont f = p.font();

    f.setPointSizeF(8.5);

    p.setFont(f);

    const auto text = [this](const Series * s, const QString & name)
        {
            if (s == nullptr || !s->got())
            {
                return QString();
            }

            const bool old = (now_ - s->stamp()) > VALUE_STALE_S;

            const QString v = old ? QString("--") : fmtValue(s->last(), 2);

            return name.isEmpty() ? v : (name + " " + v);
        };

    const QString ta = text(a_, a_name_);

    const QString tb = text(b_, b_name_);

    const QRectF band(plot.left(), 0, plot.width(), TOP_PAD - 1.0);

    if (!tb.isEmpty())
    {
        const double half = band.width() * 0.5;

        p.setPen(theme::SERIES_A);

        p.drawText(
            QRectF(band.left(), band.top(), half, band.height()),
            Qt::AlignVCenter | Qt::AlignRight, ta + "   ");

        p.setPen(theme::SERIES_B);

        p.drawText(
            QRectF(band.left() + half, band.top(), half, band.height()),
            Qt::AlignVCenter | Qt::AlignRight, tb);
    }
    else
    {
        p.setPen(theme::SERIES_SINGLE);

        p.drawText(band, Qt::AlignVCenter | Qt::AlignRight, ta);
    }
}


void PlotStrip::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), theme::PANEL_BG);

    const double bottom_pad = show_x_axis_
        ? (axis::TICK_LEN + axis::X_TICK_H + axis::X_LABEL_H)
        : BASE_PAD;

    // width()/height() 는 int 다. 1.0 과 그대로 섞으면 std::max 추론이 깨진다.
    const QRectF plot(
        axis::LEFT_MARGIN, TOP_PAD,
        std::max(1.0, static_cast<double>(width() - axis::LEFT_MARGIN - RIGHT_PAD)),
        std::max(1.0, height() - TOP_PAD - bottom_pad));

    double lo = 0.0;

    double hi = 1.0;

    computeRange(&lo, &hi);

    const double t0 = now_ - window_;

    const std::vector<double> yt = axis::niceTicks(lo, hi, 4);

    const std::vector<double> xt = axis::niceTicks(t0, now_, 6);

    // --- 격자 ---
    axis::drawGrid(p, plot, xt, t0, now_, yt, lo, hi, 77);   // viz alpha 0.3

    // --- 0 선. 격자보다 진하게 (부호가 바뀌는 지점이 곧 판단 기준) ---
    if (lo < 0.0 && hi > 0.0)
    {
        const double y =
            plot.bottom() - (0.0 - lo) / (hi - lo) * plot.height();

        p.setPen(QPen(theme::ZERO_LINE, 1.2));

        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }

    // --- 계열 ---
    drawSeries(p, a_, theme::SERIES_A, plot, lo, hi);

    drawSeries(p, b_, theme::SERIES_B, plot, lo, hi);

    // --- 축 ---
    axis::drawFrame(p, plot);

    axis::drawYAxis(p, plot, yt, lo, hi, y_label_);

    if (show_x_axis_)
    {
        axis::drawXAxis(p, plot, xt, t0, now_, "t [s]");
    }

    // --- 현재값 ---
    drawReadout(p, plot);

    // --- 안내 문구 ---
    if (!notice_.isEmpty())
    {
        QFont f = p.font();

        f.setPointSizeF(9.0);

        p.setFont(f);

        p.setPen(theme::FAULT);

        p.drawText(plot, Qt::AlignCenter, notice_);
    }
}

}  // namespace kau_gui
