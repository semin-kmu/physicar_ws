#include "kau_gui/status_bar.hpp"

#include <algorithm>
#include <cmath>

#include <QPainter>
#include <QSizePolicy>

#include "kau_gui/theme.hpp"


namespace kau_gui
{

namespace
{

constexpr int CELL_W   = 176;   // 노드명이 긴 편이라 넉넉히
constexpr int CELL_H   = 34;
constexpr int PAD      = 8;
constexpr int BANNER_H = 22;
constexpr int LAMP_R   = 5;


QColor lampColor(Health h)
{
    switch (h)
    {
        case Health::OK:
            return theme::OK;

        case Health::STALE:
            return theme::FAULT;

        case Health::ABSENT:
        default:
            return theme::ABSENT;
    }
}

}  // namespace


StatusBar::StatusBar(QWidget * parent)
: QWidget(parent)
{
    setAutoFillBackground(false);

    setMinimumHeight(BANNER_H + PAD * 2 + CELL_H * 2);

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}


void StatusBar::setNodes(const std::vector<NodeStatus> * nodes)
{
    nodes_ = nodes;

    // 창을 좁히면 열이 줄어 행이 늘어난다. 높이를 안 늘리면 아래 행이
    // 통째로 잘려 노드가 사라진 것처럼 보인다.
    const int want =
        (banner_.isEmpty() ? PAD : BANNER_H + 2) + rowsNeeded() * CELL_H + PAD;

    if (height() != want)
    {
        setFixedHeight(want);
    }
}


void StatusBar::setBanner(const QString & text)
{
    banner_ = text;
}


int StatusBar::columns() const
{
    const int usable = std::max(1, width() - PAD * 2);

    return std::max(1, usable / CELL_W);
}


int StatusBar::rowsNeeded() const
{
    if (nodes_ == nullptr || nodes_->empty())
    {
        return 1;
    }

    const int cols = columns();

    return static_cast<int>((nodes_->size() + cols - 1) / cols);
}


void StatusBar::paintEvent(QPaintEvent *)
{
    QPainter p(this);

    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), theme::PANEL_BG);

    p.setPen(QPen(theme::BORDER, 1.0));

    p.drawLine(0, height() - 1, width(), height() - 1);

    int top = PAD;


    // --- 배너 ---
    if (!banner_.isEmpty())
    {
        const QRectF box(PAD, 2, width() - PAD * 2, BANNER_H - 4);

        p.setPen(Qt::NoPen);

        QColor bg = theme::FAULT;

        bg.setAlpha(45);

        p.setBrush(bg);

        p.drawRoundedRect(box, 3, 3);

        QFont bf = p.font();

        bf.setPointSizeF(9.0);

        bf.setBold(true);

        p.setFont(bf);

        p.setPen(theme::FAULT);

        p.drawText(box, Qt::AlignCenter, banner_);

        top = BANNER_H + 2;
    }


    if (nodes_ == nullptr || nodes_->empty())
    {
        QFont f = p.font();

        f.setPointSizeF(9.0);

        p.setFont(f);

        p.setPen(theme::TEXT_DIM);

        p.drawText(
            QRectF(PAD, top, width() - PAD * 2, CELL_H),
            Qt::AlignVCenter | Qt::AlignLeft,
            "감시 대상 없음 (watch.names 미설정)");

        return;
    }


    const int cols = columns();

    // 남는 폭을 셀에 고루 나눠 준다. 고정폭으로 두면 오른쪽이 비어 보인다.
    const double cell_w =
        static_cast<double>(width() - PAD * 2) / static_cast<double>(cols);

    QFont name_font = p.font();

    name_font.setPointSizeF(9.0);

    QFont hz_font = p.font();

    hz_font.setPointSizeF(8.5);


    for (std::size_t i = 0; i < nodes_->size(); ++i)
    {
        const NodeStatus & n = (*nodes_)[i];

        const int col = static_cast<int>(i) % cols;

        const int row = static_cast<int>(i) / cols;

        const QRectF cell(
            PAD + col * cell_w, top + row * CELL_H, cell_w, CELL_H);

        // --- 1 행: 노드명 + 불빛 ---
        const QRectF name_box(
            cell.left(), cell.top() + 2, cell.width() - 18, 15);

        p.setFont(name_font);

        p.setPen(n.health == Health::ABSENT ? theme::TEXT_DIM : theme::TEXT);

        // 이름이 길면 잘라 준다. 셀을 넘치면 옆 노드와 붙어 읽을 수 없다.
        const QString elided =
            p.fontMetrics().elidedText(
                QString::fromStdString(n.name), Qt::ElideRight,
                static_cast<int>(name_box.width()));

        p.drawText(name_box, Qt::AlignVCenter | Qt::AlignLeft, elided);

        const QColor lamp = lampColor(n.health);

        p.setPen(Qt::NoPen);

        // 바깥 옅은 링 + 안쪽 원. 작은 점만으로는 색 구분이 잘 안 된다.
        QColor halo = lamp;

        halo.setAlpha(45);

        p.setBrush(halo);

        p.drawEllipse(
            QPointF(cell.right() - 12, name_box.center().y()),
            LAMP_R + 3, LAMP_R + 3);

        p.setBrush(lamp);

        p.drawEllipse(
            QPointF(cell.right() - 12, name_box.center().y()),
            LAMP_R, LAMP_R);

        // --- 2 행: Hz ---
        p.setFont(hz_font);

        p.setPen(theme::TEXT_DIM);

        QString hz_text;

        if (!n.rate_checked)
        {
            // 주기 판정 대상이 아니다. 0 Hz 로 오해하지 않도록 '-'.
            hz_text = "-";
        }
        else if (n.hz < 0.0)
        {
            hz_text = "-- Hz";
        }
        else
        {
            hz_text = QString("%1 Hz").arg(n.hz, 0, 'f', 1);
        }

        p.drawText(
            QRectF(cell.left(), cell.top() + 17, cell.width() - 18, 14),
            Qt::AlignVCenter | Qt::AlignRight, hz_text);
    }
}

}  // namespace kau_gui
