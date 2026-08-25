// ====================================================================
// kau_lane_detection_node.cpp
//
// 진입점. 클래스 선언은 헤더, 구현은 kau_lane_detection.cpp.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <opencv2/core.hpp>

#include <cstdlib>
#include <memory>


// ====================================================================
// Main
// ====================================================================

int main(
    int argc,
    char * argv[])
{
    rclcpp::init(
        argc,
        argv
    );


    // ----------------------------------------------------------------
    // OpenCV 워커 스레드 수
    //
    // 기본값은 "코어 수만큼" 이다. 그러면 remap / warpPerspective /
    // cvtColor / inRange / morphologyEx 를 부를 때마다 4코어짜리
    // Pi 5 의 **전 코어**를 순간 점유한다. 그 사이 제어 루프
    // (local_path_planner / control) 가 스케줄에서 밀려 주기가
    // 들쭉날쭉해진다.
    //
    // 480x360 은 한 코어로도 충분히 작아서 분할 이득보다 스레드를
    // 깨우는 비용이 크다. 1 로 고정해 이 노드를 한 코어 안에
    // 가둔다 — 지각이 조금 느려지더라도 제어 주기가 흔들리지 않는
    // 쪽이 차량 거동에 낫다.
    //
    // 노드 파라미터로 두지 않은 이유: setNumThreads 는 프로세스
    // 전역이고 스핀 시작 전에 정해져야 한다. 벤치마크할 때만
    // 환경변수로 덮는다:  KAU_OPENCV_THREADS=4 ros2 run ...
    {
        int cv_threads = 1;

        if (const char * env = std::getenv("KAU_OPENCV_THREADS"))
        {
            cv_threads = std::atoi(env);
        }

        cv::setNumThreads(cv_threads);
    }


    auto node =
        std::make_shared<KauLaneDetectionNode>();


    // ----------------------------------------------------------------
    // 마지막 방어선
    //
    // 콜백 각각은 생성자에서 guardCallback 으로 감싸 두었으므로
    // 보통은 여기까지 예외가 올라오지 않는다. 그래도 감싸는 이유는
    // 콜백 **밖**(executor 내부, TF 리스너, DDS 계층)에서 던져지는
    // 예외가 남아 있기 때문이다.
    //
    // 이건 노드를 살려내지 못한다 — spin() 이 빠져나온 뒤에는 이미
    // 스핀이 끝난 것이라 복구가 아니다. 목적은 하나다:
    // **"조용히 죽는 것"을 "로그를 남기고 죽는 것"으로 바꾸는 것.**
    // 대회 중에 차가 원인 불명으로 서는 것보다는 낫다.
    // ----------------------------------------------------------------
    try
    {
        rclcpp::spin(
            node
        );
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "치명적 예외로 종료합니다: %s",
            e.what()
        );
    }
    catch (...)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "알 수 없는 치명적 예외로 종료합니다."
        );
    }


    rclcpp::shutdown();


    return 0;
}