// ====================================================================
// plot_strip.hpp
//
// 좌측 패널의 시계열 한 줄. 최대 두 계열을 겹쳐 그린다.
//
// 데시메이션 규약: 픽셀 열마다 그 열에 걸치는 표본의 min/max 를 세로선
// 으로 긋는다. 단순 솎아내기를 하면 조향 떨림 · cte 스파이크처럼 한두
// 표본짜리 사건이 통째로 사라지는데, 그건 디버깅에서 가장 보고 싶은
// 것이다. 그리는 양도 표본 수가 아니라 위젯 폭에 비례하게 된다.
// ====================================================================

#ifndef KAU_GUI__PLOT_STRIP_HPP_
#define KAU_GUI__PLOT_STRIP_HPP_

#include <QColor>
#include <QRectF>
#include <QString>
#include <QWidget>

#include "kau_gui/types.hpp"


class QPainter;


namespace kau_gui
{

class PlotStrip : public QWidget
{
    Q_OBJECT

public:
    PlotStrip(
        const QString & title, const QString & unit, QWidget * parent = nullptr);

    // 계열 이름. 하나만 쓰면 b_name 을 비운다.
    void setSeriesNames(const QString & a_name, const QString & b_name);

    // y 축이 최소한 이만큼은 벌어지게 둔다. 값이 0 근처에서 미동할 때
    // 축이 과도하게 확대돼 노이즈가 신호처럼 보이는 것을 막는다.
    void setMinSpan(double span);

    // 0 을 항상 축 안에 포함시킨다 (오차 플롯용).
    void setIncludeZero(bool on);

    // window_s 는 x 축 길이. now 는 오른쪽 끝 시각.
    void setData(
        const Series * a, const Series * b, double now, double window_s);

    // 데이터가 있어도 이 문구가 있으면 위에 겹쳐 띄운다 (예: 추종 불가).
    void setNotice(const QString & text);

protected:
    void paintEvent(QPaintEvent * e) override;

private:
    void computeRange(double * lo, double * hi) const;

    void drawSeries(
        QPainter & p, const Series * s, const QColor & c, const QRectF & plot,
        double lo, double hi) const;

    QString title_;
    QString unit_;
    QString a_name_;
    QString b_name_;
    QString notice_;

    const Series * a_ = nullptr;
    const Series * b_ = nullptr;

    double now_      = 0.0;
    double window_   = 30.0;
    double min_span_ = 1.0;

    bool include_zero_ = true;
};

}  // namespace kau_gui

#endif  // KAU_GUI__PLOT_STRIP_HPP_
