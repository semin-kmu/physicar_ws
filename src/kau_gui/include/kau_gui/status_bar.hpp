// ====================================================================
// status_bar.hpp
//
// 최상단 노드 상태 패널. 좌우 분할과 무관하게 가로 전체를 쓴다.
//
// 셀 하나 = 한 노드.
//     [노드명            ●]
//     [          12.4 Hz  ]
//
// 감시 대상은 bringup.yaml 이 기동하는 노드 그대로다. 목록을 config 에
// 고정해 두는 이유는, 자동 탐색으로는 **떠 있지 않은 노드**를 보여줄 수
// 없어 정작 필요한 빨간불이 안 켜지기 때문이다.
//
// 색: 초록 정상 / 빨강 노드는 살아있으나 토픽 끊김 / 회색 미기동.
// 등급 색은 신호등과 무관한 별도 배색이다 (docs/09 section 11-2).
// ====================================================================

#ifndef KAU_GUI__STATUS_BAR_HPP_
#define KAU_GUI__STATUS_BAR_HPP_

#include <vector>

#include <QString>
#include <QWidget>

#include "kau_gui/types.hpp"


namespace kau_gui
{

class StatusBar : public QWidget
{
    Q_OBJECT

public:
    explicit StatusBar(QWidget * parent = nullptr);

    void setNodes(const std::vector<NodeStatus> * nodes);

    // 상단 배너에 띄울 경고. 비면 배너를 그리지 않는다.
    void setBanner(const QString & text);

protected:
    void paintEvent(QPaintEvent * e) override;

private:
    int columns() const;

    int rowsNeeded() const;

    const std::vector<NodeStatus> * nodes_ = nullptr;

    QString banner_;
};

}  // namespace kau_gui

#endif  // KAU_GUI__STATUS_BAR_HPP_
