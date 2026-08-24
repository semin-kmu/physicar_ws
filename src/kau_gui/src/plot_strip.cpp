#include "kau_gui/plot_strip.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <QPainter>
#include <QPolygonF>

#include "kau_gui/theme.hpp"


namespace kau_gui
{

namespace
{

constexpr int LEFT_PAD   = 46;   // y 축 눈금 글자 자리
constexpr int RIGHT_PAD  = 8;
constexpr int TOP_PAD    = 20;   // 제목 줄
constexpr int BOTTOM_PAD = 6;

// 마지막 표본이 이보다 오래됐으면 "값 없음" 으로 본다.
constexpr double VALUE_STALE_S = 1.0;


QString fmtValue(double v, int digits)
{
    return QString::number(v, 'f', digits);
}

}  // namespace


PlotStrip::PlotStrip(
    const QString & title, const QString & unit, QWidget * parent)
: QWidget(parent),
  title_(title),
  unit_(unit)
{
    setMinimumHeight(78);

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

        for (const Sample & p : s->data())
        {
            if (p.t < t0)
            {
                continue;
            }

            mn = std::min(mn, p.v);

            mx = std::max(mx, p.v);
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

    // 최소 폭 보장 + 위아래 여유
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

    p.setPen(QPen(c, 1.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

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


void PlotStrip::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), theme::PANEL_BG);

    const QRectF plot(
        LEFT_PAD, TOP_PAD,
        std::max(1.0, width() - LEFT_PAD - RIGHT_PAD),
        std::max(1.0, height() - TOP_PAD - BOTTOM_PAD));

    double lo = 0.0;

    double hi = 1.0;

    computeRange(&lo, &hi);

    // --- 테두리 ---
    p.setPen(QPen(theme::BORDER, 1.0));

    p.setBrush(Qt::NoBrush);

    p.drawRect(plot);

    // --- 0 선 ---
    if (lo < 0.0 && hi > 0.0)
    {
        const double y =
            plot.bottom() - (0.0 - lo) / (hi - lo) * plot.height();

        p.setPen(QPen(theme::ZERO_LINE, 1.0, Qt::DashLine));

        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }

    // --- y 눈금 (상/하한만) ---
    QFont f = p.font();

    f.setPointSizeF(8.0);

    p.setFont(f);

    p.setPen(theme::TEXT_DIM);

    const int digits = (std::max(std::abs(lo), std::abs(hi)) < 10.0) ? 2 : 1;

    p.drawText(
        QRectF(0, plot.top() - 6, LEFT_PAD - 6, 12),
        Qt::AlignRight | Qt::AlignVCenter, fmtValue(hi, digits));

    p.drawText(
        QRectF(0, plot.bottom() - 6, LEFT_PAD - 6, 12),
        Qt::AlignRight | Qt::AlignVCenter, fmtValue(lo, digits));

    // --- 계열 ---
    drawSeries(p, a_, theme::SERIES_A, plot, lo, hi);

    drawSeries(p, b_, theme::SERIES_B, plot, lo, hi);

    // --- 제목 + 현재값 ---
    f.setPointSizeF(9.0);

    f.setBold(true);

    p.setFont(f);

    p.setPen(theme::TEXT);

    p.drawText(
        QRectF(LEFT_PAD, 1, plot.width() * 0.5, TOP_PAD - 2),
        Qt::AlignVCenter | Qt::AlignLeft,
        unit_.isEmpty() ? title_ : (title_ + "  [" + unit_ + "]"));

    f.setBold(false);

    p.setFont(f);

    // 오른쪽 위에 현재값. 그래프를 읽지 않아도 숫자는 바로 보여야 한다.
    QString right;

    const auto valueText = [this](const Series * s, const QString & name)
        {
            if (s == nullptr || !s->got())
            {
                return QString();
            }

            const bool old = (now_ - s->stamp()) > VALUE_STALE_S;

            const QString v = old ? QString("--") : fmtValue(s->last(), 2);

            return name.isEmpty() ? v : (name + " " + v);
        };

    const QString ta = valueText(a_, a_name_);

    const QString tb = valueText(b_, b_name_);

    if (!ta.isEmpty() && !tb.isEmpty())
    {
        right = ta + "   " + tb;
    }
    else
    {
        right = ta + tb;
    }

    // 계열 색과 글자 색을 맞추면 어느 숫자가 어느 선인지 즉시 읽힌다.
    if (!ta.isEmpty() && !tb.isEmpty())
    {
        const QRectF box(
            LEFT_PAD + plot.width() * 0.35, 1,
            plot.width() * 0.65, TOP_PAD - 2);

        const int mid = static_cast<int>(box.width() * 0.5);

        p.setPen(theme::SERIES_A);

        p.drawText(
            QRectF(box.left(), box.top(), mid, box.height()),
            Qt::AlignVCenter | Qt::AlignRight, ta + "  ");

        p.setPen(theme::SERIES_B);

        p.drawText(
            QRectF(box.left() + mid, box.top(), mid, box.height()),
            Qt::AlignVCenter | Qt::AlignRight, tb);
    }
    else
    {
        p.setPen(theme::SERIES_SINGLE);

        p.drawText(
            QRectF(
                LEFT_PAD + plot.width() * 0.4, 1,
                plot.width() * 0.6, TOP_PAD - 2),
            Qt::AlignVCenter | Qt::AlignRight, right);
    }

    // --- 안내 문구 ---
    if (!notice_.isEmpty())
    {
        p.setPen(theme::WARN);

        p.drawText(plot, Qt::AlignCenter, notice_);
    }
}

}  // namespace kau_gui
