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


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}