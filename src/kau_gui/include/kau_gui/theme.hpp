// ====================================================================
// theme.hpp
//
// 배색 한 곳. 색을 코드 여기저기에 흩어 두면 범례와 실제 선 색이
// 어긋나는 사고가 난다.
//
// 규약은 KAU_AMET_Test 의 sim_common/viz.py 를 그대로 따른다.
// 시뮬과 실차 GUI 를 나란히 놓고 볼 일이 많으므로 같은 색이 같은 것을
// 가리켜야 한다.
//
//     pg.setConfigOptions(background="w", foreground="k")
//     -> 흰 바탕 / 검정 전경. matplotlib tab10 팔레트
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
const QColor WINDOW_BG              (250, 250, 250);
const QColor MAP_BG                 (255, 255, 255);
const QColor PANEL_BG               (255, 255, 255);
const QColor PANEL_BG_TRANSLUCENT   (255, 255, 255, 220);  // viz.hud fill
const QColor GRID                   (204, 204, 204);       // alpha 는 그릴 때
const QColor BORDER                 (170, 170, 170);
const QColor AXIS                   ( 60,  60,  60);

// --- 글자 ---
const QColor TEXT                   ( 20,  20,  20);       // foreground="k"
const QColor TEXT_DIM               (120, 120, 120);

// --- 맵 표시물 (viz.py COL_* 그대로) ---
const QColor PATH_GLOBAL            (0x1f, 0x77, 0xb4);    // COL_GLOBAL
const QColor PATH_LANE              (0x2c, 0xa0, 0x2c);    // COL_LANE
const QColor PATH_LOCAL             (0xd6, 0x27, 0x28);    // COL_LOCAL
const QColor VEHICLE                (0xd6, 0x27, 0x28);    // COL_CAR
const QColor OBSTACLE               (0x7f, 0x2f, 0x2f);    // COL_OBS
const QColor LOOKAHEAD              (0xff, 0x7f, 0x0e);    // COL_CLICK
const QColor SCAN                   (0x8c, 0x8c, 0x8c);    // 시뮬엔 없음. 참값 회색 계열
const QColor SCAN_STALE             (0xd0, 0xd0, 0xd0);
const QColor MAP_BORDER             (0x00, 0x00, 0x00);    // draw_map_border

// --- 플롯 (viz.py COL_REF / OVERLAY) ---
const QColor SERIES_A               (0x1f, 0x77, 0xb4);    // COL_REF
const QColor SERIES_B               (0xff, 0x7f, 0x0e);    // tab10 orange
const QColor SERIES_SINGLE          (0x1f, 0x77, 0xb4);
const QColor ZERO_LINE              (204, 204, 204);       // viz.stack_plots

// --- 등급 ---
const QColor OK                     (0x2c, 0xa0, 0x2c);
const QColor WARN                   (0xff, 0x7f, 0x0e);
const QColor FAULT                  (0xd6, 0x27, 0x28);
const QColor ABSENT                 (0xb0, 0xb0, 0xb0);

}  // namespace theme
}  // namespace kau_gui

#endif  // KAU_GUI__THEME_HPP_
