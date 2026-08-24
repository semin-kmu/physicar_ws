#include "kau_gui/main_window.hpp"

#include <algorithm>

#include <QKeyEvent>
#include <QLabel>
#include <QPalette>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "kau_gui/map_view.hpp"
#include "kau_gui/plot_axis.hpp"
#include "kau_gui/plot_strip.hpp"
#include "kau_gui/status_bar.hpp"
#include "kau_gui/theme.hpp"


namespace kau_gui
{

MainWindow::MainWindow(std::shared_ptr<RosBridge> bridge)
: bridge_(std::move(bridge))
{
    history_s_ = bridge_->get_parameter("history_s").as_double();

    buildUi();

    const double hz = std::max(1.0, bridge_->renderHz());

    timer_ = new QTimer(this);

    connect(timer_, &QTimer::timeout, this, &MainWindow::onTick);

    timer_->start(static_cast<int>(1000.0 / hz));

    setWindowTitle("kau_gui · 주행 디버깅 (관측 전용)");

    resize(1500, 900);
}


void MainWindow::buildUi()
{
    QWidget * central = new QWidget(this);

    QVBoxLayout * root = new QVBoxLayout(central);

    root->setContentsMargins(0, 0, 0, 0);

    root->setSpacing(0);


    // --- 상단 ---
    status_ = new StatusBar(central);

    root->addWidget(status_, 0);


    // --- 좌우 ---
    QSplitter * split = new QSplitter(Qt::Horizontal, central);

    split->setHandleWidth(3);

    split->setChildrenCollapsible(false);


    // 좌: 플롯 5 개
    QWidget * left = new QWidget(split);

    QVBoxLayout * lv = new QVBoxLayout(left);

    lv->setContentsMargins(6, 6, 6, 6);

    lv->setSpacing(6);

    // 이름 · 단위 표기는 viz.py PANELS 를 그대로 따른다.
    plot_speed_ = new PlotStrip("speed [m/s]", left);

    plot_speed_->setSeriesNames("target", "real");

    plot_speed_->setMinSpan(0.5);

    plot_steer_ = new PlotStrip("steer [deg]", left);

    plot_steer_->setSeriesNames("raw", "cmd");

    plot_steer_->setMinSpan(10.0);

    plot_ld_ = new PlotStrip("lookahead [cm]", left);

    plot_ld_->setSeriesNames("Ld", "");

    plot_ld_->setMinSpan(50.0);

    plot_heading_ = new PlotStrip("heading error [deg]", left);

    plot_heading_->setSeriesNames("", "");

    plot_heading_->setMinSpan(10.0);

    plot_cte_ = new PlotStrip("cross track error [cm]", left);

    plot_cte_->setSeriesNames("", "");

    plot_cte_->setMinSpan(20.0);

    for (PlotStrip * s :
        {plot_speed_, plot_steer_, plot_ld_, plot_heading_, plot_cte_})
    {
        lv->addWidget(s, 1);
    }

    // x 축 눈금은 맨 아래 하나만. 전 플롯이 같은 시간창을 쓰므로 축을
    // 반복할 이유가 없다 (viz.stack_plots 규약).
    plot_cte_->setShowXAxis(true);

    hint_ = new QLabel(
        "space 일시정지   ·   r 뷰 리셋   ·   f 차량 추종   ·   "
        "휠 줌 / 드래그 팬", left);

    hint_->setStyleSheet(
        QString("color: %1; font-size: 11px; padding: 2px 0 0 %2px;")
            .arg(theme::TEXT_DIM.name())
            .arg(axis::LEFT_MARGIN));

    lv->addWidget(hint_, 0);


    // 우: 맵
    map_ = new MapView(split);

    map_->setVehicleSize(
        bridge_->vehicleLengthM(), bridge_->vehicleWidthM());

    map_->setDisplayUnit(
        QString::fromStdString(
            bridge_->get_parameter("map.display_unit").as_string()));

    split->addWidget(left);

    split->addWidget(map_);

    // viz.py 의 setColumnStretchFactor(0, 95) / (1, 125)
    split->setStretchFactor(0, 95);

    split->setStretchFactor(1, 125);

    split->setSizes({640, 860});

    root->addWidget(split, 1);


    setCentralWidget(central);

    // 팔레트로 배경을 깔아 둔다. 위젯마다 fillRect 하지만 splitter 손잡이
    // 같은 기본 위젯은 팔레트를 따른다.
    QPalette pal = palette();

    pal.setColor(QPalette::Window, theme::WINDOW_BG);

    pal.setColor(QPalette::WindowText, theme::TEXT);

    setPalette(pal);

    central->setAutoFillBackground(true);

    central->setPalette(pal);
}


void MainWindow::onTick()
{
    // 일시정지는 표시만 멈춘다. 수신은 계속되므로 재개하면 그동안의
    // 이력이 그대로 남아 있다 (docs/09 section 11-3).
    if (paused_)
    {
        return;
    }

    snap_ = bridge_->snapshot();


    // --- 상단 ---
    // 배너를 먼저 정한다. setNodes 가 배너 유무까지 넣어 높이를 잡으므로
    // 순서가 뒤집히면 한 프레임 늦게 반영된다.
    // 전부 미기동이면 개별 불빛보다 "연결 자체가 안 됐다" 가 먼저다.
    const bool any_alive =
        std::any_of(
            snap_.nodes.begin(), snap_.nodes.end(),
            [](const NodeStatus & n)
            {
                return n.health != Health::ABSENT;
            });

    if (!snap_.nodes.empty() && !any_alive)
    {
        status_->setBanner(
            "연결 끊김 · 감시 대상 노드가 하나도 보이지 않는다 "
            "(ROS_DOMAIN_ID / zenoh 라우터 확인)");
    }
    else if (!snap_.pose.valid)
    {
        status_->setBanner("측위 없음 · map -> base TF 미수신 또는 지연");
    }
    else
    {
        status_->setBanner(QString());
    }

    status_->setNodes(&snap_.nodes);

    status_->update();


    // --- 맵 ---
    map_->setSnapshot(&snap_);

    map_->update();


    // --- 플롯 ---
    const double now = snap_.now;

    plot_speed_->setData(
        &snap_.speed_target, &snap_.speed_real, now, history_s_);

    plot_steer_->setData(
        &snap_.steer_raw, &snap_.steer_cmd, now, history_s_);

    plot_ld_->setData(&snap_.lookahead_cm, nullptr, now, history_s_);

    plot_heading_->setData(&snap_.heading_err_deg, nullptr, now, history_s_);

    plot_cte_->setData(&snap_.cross_track_cm, nullptr, now, history_s_);

    // /debug/steer 가 없으면 네 플롯이 통째로 빈다. 왜 비었는지 화면에
    // 적어 두지 않으면 GUI 버그로 오해하게 된다.
    const QString notice =
        !snap_.steer_debug_alive
            ? QString("/debug/steer 미수신 · steer_controller 의 "
                      "debug.enabled 확인")
        : !snap_.tracking_ok
            ? QString("추종 불가 · 조향 0 발행 중")
            : QString();

    for (PlotStrip * s : {plot_steer_, plot_ld_, plot_heading_, plot_cte_})
    {
        s->setNotice(notice);

        s->update();
    }

    plot_speed_->update();
}


void MainWindow::keyPressEvent(QKeyEvent * e)
{
    switch (e->key())
    {
        case Qt::Key_Space:
            paused_ = !paused_;

            setWindowTitle(
                paused_
                    ? "kau_gui · 주행 디버깅 (관측 전용) — 일시정지"
                    : "kau_gui · 주행 디버깅 (관측 전용)");

            return;

        case Qt::Key_R:
            map_->resetView();

            return;

        case Qt::Key_F:
            map_->setFollow(!map_->follow());

            return;

        default:
            QMainWindow::keyPressEvent(e);
    }
}

}  // namespace kau_gui
