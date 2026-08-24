// ====================================================================
// main.cpp
//
// rclcpp executor 를 별도 스레드에서 돌리고, Qt 는 메인 스레드에서 돈다.
// 둘 사이의 유일한 접점은 RosBridge::snapshot() 이고 그 안에서만 lock 을
// 잡는다 (docs/09 section 12).
//
// 종료 순서가 중요하다. Qt 루프가 끝난 뒤 executor 를 멈추고 join 해야
// 노드가 살아 있는 채로 소멸되는 일이 없다.
// ====================================================================

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "kau_gui/main_window.hpp"
#include "kau_gui/ros_bridge.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Qt 는 --ros-args 를 모른다. rclcpp 가 걷어내고 남은 인자만 넘긴다.
    // 문자열 실체와 포인터 배열 모두 QApplication 보다 오래 살아야 한다.
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.empty())
    {
        args.push_back("kau_gui_node");
    }

    std::vector<char *> qt_argv;

    qt_argv.reserve(args.size() + 1);

    for (std::string & s : args)
    {
        qt_argv.push_back(s.data());
    }

    qt_argv.push_back(nullptr);

    int qt_argc = static_cast<int>(args.size());

    QApplication app(qt_argc, qt_argv.data());


    std::shared_ptr<kau_gui::RosBridge> bridge;

    try
    {
        bridge = std::make_shared<kau_gui::RosBridge>();
    }
    catch (const std::exception & e)
    {
        // 설정 오류(watch 배열 길이 불일치 등)는 여기서 걸린다.
        // 창을 띄우고 나서 죽는 것보다 이유를 남기고 즉시 끝내는 편이 낫다.
        RCLCPP_FATAL(
            rclcpp::get_logger("kau_gui"), "[kau_gui] 기동 실패: %s",
            e.what());

        rclcpp::shutdown();

        return 1;
    }


    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(bridge);

    std::thread spin_thread(
        [&exec]()
        {
            exec.spin();
        });


    int rc = 0;

    {
        kau_gui::MainWindow win(bridge);

        win.show();

        rc = app.exec();
    }


    exec.cancel();

    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    exec.remove_node(bridge);

    bridge.reset();

    rclcpp::shutdown();

    return rc;
}
