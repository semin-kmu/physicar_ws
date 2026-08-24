// ====================================================================
// plot_axis.hpp
//
// 축 · 눈금 · 그리드. pyqtgraph 의 AxisItem 이 공짜로 주던 부분을
// QPainter 로 직접 구현한 것이다.
//
// 이게 없으면 값을 눈으로 읽을 수 없다. 플롯을 "그림" 이 아니라
// "계측기" 로 만드는 것이 이 파일의 전부다.
//
// 눈금은 1-2-5 계열 nice number. 축 범위를 정확히 맞추는 것보다
// 눈금 값이 읽기 쉬운 것이 우선이다.
// ====================================================================

#ifndef KAU_GUI__PLOT_AXIS_HPP_
#define KAU_GUI__PLOT_AXIS_HPP_

#include <vector>

#include <QRectF>
#include <QString>

class QPainter;


namespace kau_gui
{
namespace axis
{

// 여백 규약. 축 라벨과 눈금 글자가 들어갈 자리다.
constexpr int TICK_LEN   = 4;
constexpr int Y_LABEL_W  = 16;   // 세로로 눕힌 y 축 이름
constexpr int Y_TICK_W   = 42;   // y 눈금 숫자
constexpr int X_TICK_H   = 15;   // x 눈금 숫자
constexpr int X_LABEL_H  = 15;   // x 축 이름

constexpr int LEFT_MARGIN = Y_LABEL_W + Y_TICK_W;


// [lo, hi] 안의 1-2-5 계열 눈금 값. want 는 목표 개수(정확하지 않아도 된다).
std::vector<double> niceTicks(double lo, double hi, int want);

// 눈금 간격에 맞춰 필요한 소수 자릿수만 남긴 문자열.
QString tickLabel(double v, double step);

// 눈금 목록의 간격. 1 개 이하면 0.
double stepOf(const std::vector<double> & ticks);


// 격자만 그린다 (테두리 · 숫자 제외).
void drawGrid(
    QPainter & p, const QRectF & plot,
    const std::vector<double> & xt, double x0, double x1,
    const std::vector<double> & yt, double y0, double y1,
    int alpha);

// plot 왼쪽 바깥에 y 눈금 숫자 + (name 이 있으면) 눕힌 축 이름.
void drawYAxis(
    QPainter & p, const QRectF & plot, const std::vector<double> & yt,
    double y0, double y1, const QString & name);

// plot 아래쪽 바깥에 x 눈금 숫자 + (name 이 있으면) 축 이름.
void drawXAxis(
    QPainter & p, const QRectF & plot, const std::vector<double> & xt,
    double x0, double x1, const QString & name);

// 축 테두리.
void drawFrame(QPainter & p, const QRectF & plot);

}  // namespace axis
}  // namespace kau_gui

#endif  // KAU_GUI__PLOT_AXIS_HPP_
