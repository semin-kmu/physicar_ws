// ====================================================================
// kau_lane_detection_node.cpp
//
// 진입점. 클래스 선언은 헤더, 구현은 kau_lane_detection.cpp.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

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


    auto node =
        std::make_shared<KauLaneDetectionNode>();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}