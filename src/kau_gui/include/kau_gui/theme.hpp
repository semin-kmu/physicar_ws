// ====================================================================
// theme.hpp
//
// 배색 한 곳. 색을 코드 여기저기에 흩어 두면 범례와 실제 선 색이
// 어긋나는 사고가 난다.
//
// 어두운 바탕을 쓰는 이유는 취향이 아니라, 점유격자 맵(흰 자유공간 ·
// 검은 벽) 위에 색선을 얹었을 때 대비가 가장 크기 때문이다.
//
// 등급 색(초록/주황/빨강)은 신호등과 무관한 별도 배색이다 (docs/09 11-2).
// ====================================================================

#ifndef KAU_GUI__THEME_HPP_
#define KAU_GUI__THEME_HPP_

#include <QColor>


namespace kau_gui
{
namespace theme
{

// --- 바탕 ---
const QColor WINDOW_BG              (24, 26, 30);
const QColor MAP_BG                 (18, 20, 24);
const QColor PANEL_BG               (30, 33, 38);
const QColor PANEL_BG_TRANSLUCENT   (22, 24, 28, 210);
const QColor GRID                   (52, 57, 65);
const QColor BORDER                 (60, 66, 76);

// --- 글자 ---
const QColor TEXT                   (222, 228, 236);
const QColor TEXT_DIM               (128, 138, 152);

// --- 맵 표시물 ---
const QColor PATH_GLOBAL            (110, 120, 140);   // 회청색, 배경 취급
const QColor PATH_LANE              (120, 200, 255);   // 하늘
const QColor PATH_LOCAL             (120, 230, 150);   // 초록. 지금 따라가는 것
const QColor SCAN                   (235, 232, 140);   // 노랑
const QColor SCAN_STALE             (110, 108, 70);
const QColor OBSTACLE               (255, 120, 60);    // 주황
const QColor VEHICLE                (255, 255, 255);
const QColor LOOKAHEAD              (255, 105, 180);   // 분홍

// --- 플롯 ---
// 한 플롯에 두 계열이 겹칠 때 쓰는 짝. 명령이 밝고 실측이 어둡다.
const QColor SERIES_A               (120, 200, 255);   // target / raw
const QColor SERIES_B               (255, 170, 90);    // real / cmd
const QColor SERIES_SINGLE          (150, 220, 170);
const QColor ZERO_LINE              (78, 86, 98);

// --- 등급 ---
const QColor OK                     (90, 200, 120);
const QColor WARN                   (240, 170, 60);
const QColor FAULT                  (235, 85, 85);
const QColor ABSENT                 (90, 96, 106);

}  // namespace theme
}  // namespace kau_gui

#endif  // KAU_GUI__THEME_HPP_
