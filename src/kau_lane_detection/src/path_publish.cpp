// ====================================================================
// path_publish.cpp
//
// KauPath / RViz 이산화 발행. path_frame_id 변환 포함.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// ================================================================
// KauPath 발행 (docs/경로_형식.md 9.5)
//
// 절차:
//   1 제어점은 이미 quintic (buildCenterlinePath 에서 승격 완료)
//   2 호길이   Gauss-Legendre 10점  -> segLength
//   3 곡률상한 14차 근              -> kappaMaxExact
//   4 퇴화검사 isRegular            -> buildCenterlinePath 7절에서 완료
//   5 flat 적재
//
// segment 는 항상 1개다 (9.3, Lane Detection 의 표준 형태).
// 이산 좌표는 만들지도 싣지도 않는다. RViz 용 이산화만
// /viz/path/lane 으로 따로 나간다.
// ================================================================

void KauLaneDetectionNode::publishLanePath(
    const LanePath & path,
    const rclcpp::Time & stamp,
    const rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr & pub)
{
    const rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr & out =
        pub ? pub : lane_path_publisher_;

    if (
        !out ||
        !path.valid
    )
    {
        return;
    }


    // 무결성: 수신측 검사(9.8)를 발행측에서 먼저 건다.
    if (
        path.ctrl.size() !=
        static_cast<std::size_t>(kau::bezier::NCTRL)
    )
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "제어점 개수가 %zu 개입니다 (기대 %d). 발행하지 않습니다.",
            path.ctrl.size(),
            kau::bezier::NCTRL
        );

        return;
    }


    // ------------------------------------------------------------
    // map 프레임 변환
    //
    // path.ctrl 은 내부적으로 항상 base_link 기준이다 (EMA/게이트/
    // debug 오버레이가 "자차가 원점" 을 전제하므로 여기서는 절대
    // 바꾸지 않는다 — 선언부 헤더 주석 참고). map 으로 낼 때는
    // 발행 직전에 사본만 옮긴다.
    //
    // TF 조회가 실패하면 이번 프레임은 건너뛴다. 낡은 map 위치로
    // 잘못된 절대좌표를 내느니, 하류가 직전 값을 들고 가는 쪽이
    // 안전하다 (다른 게이트 기각과 같은 방침).
    // ------------------------------------------------------------

    const bool to_map = (path_frame_id_ == "map");

    VehiclePose vehicle_pose;

    if (to_map)
    {
        vehicle_pose = lookupVehiclePose(stamp);

        if (!vehicle_pose.valid)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "map 변환 실패로 이번 프레임은 발행하지 않습니다."
            );

            return;
        }
    }

    const double cos_yaw = std::cos(vehicle_pose.yaw_rad);

    const double sin_yaw = std::sin(vehicle_pose.yaw_rad);


    kau_msgs::msg::KauPath msg;

    msg.header.stamp = stamp;

    msg.header.frame_id = path_frame_id_;

    msg.source = kau_msgs::msg::KauPath::SRC_LANE;

    msg.degree =
        static_cast<uint8_t>(kau::bezier::DEGREE);

    msg.is_closed = false;

    // 전역 호길이 기준점 없음 (map 으로 내도 이 경로 자체의 s 는
    // 0 부터다 — /path/global 의 전역 s 와는 별개)
    msg.s_offset = 0.0;


    // ------------------------------------------------------------
    // flat 적재. segment 1개이므로 그대로 6개.
    // ------------------------------------------------------------

    msg.ctrl_x.reserve(kau::bezier::NCTRL);

    msg.ctrl_y.reserve(kau::bezier::NCTRL);


    for (const kau::bezier::Point2 & cp : path.ctrl)
    {
        if (to_map)
        {
            // base_link [cm, X 전방 / Y 좌] -> map [m] 회전+평행이동
            // -> [cm]. 강체변환이므로 seg_length/kappa_max 는
            // (KauPath.msg 주석대로) 다시 잴 필요가 없다.
            const double x_m = cp.x * 0.01;

            const double y_m = cp.y * 0.01;

            const double map_x_m =
                vehicle_pose.x_m +
                x_m * cos_yaw - y_m * sin_yaw;

            const double map_y_m =
                vehicle_pose.y_m +
                x_m * sin_yaw + y_m * cos_yaw;

            msg.ctrl_x.push_back(map_x_m * 100.0);

            msg.ctrl_y.push_back(map_y_m * 100.0);
        }
        else
        {
            msg.ctrl_x.push_back(cp.x);

            msg.ctrl_y.push_back(cp.y);
        }
    }


    msg.seg_length = { path.length_cm };

    msg.seg_kappa_max = {
        static_cast<float>(path.kappa_max)
    };

    msg.total_length = path.length_cm;

    msg.confidence =
        static_cast<float>(path.confidence);

    msg.valid_length = path.valid_length_cm;


    out->publish(msg);


    // ------------------------------------------------------------
    // RViz 전용 이산화 (9.9)
    //
    // 구독자가 없으면 만들지 않는다. 제어 경로는 이 값을
    // 쓰지 않으므로 대전제에 저촉되지 않는다.
    //
    // nav_msgs/Path 는 ROS 표준이므로 단위가 m 다.
    // KauPath 는 팀 규약대로 cm 이므로 여기서만 100 으로 나눈다.
    // ------------------------------------------------------------

    if (
        !viz_path_publisher_ ||
        viz_path_publisher_->get_subscription_count() == 0
    )
    {
        return;
    }


    constexpr int VIZ_SAMPLES = 40;

    nav_msgs::msg::Path viz;

    viz.header = msg.header;

    viz.poses.reserve(VIZ_SAMPLES);


    for (int i = 0; i < VIZ_SAMPLES; ++i)
    {
        const double u =
            static_cast<double>(i) /
            static_cast<double>(VIZ_SAMPLES - 1);

        const kau::bezier::Point2 p =
            kau::bezier::evalSeg(path.ctrl, u);

        // path.ctrl(base_link) 기준 헤딩. map 표시용 헤딩은 아래에서
        // 차체 yaw 를 더해 만든다.
        const double th =
            kau::bezier::heading(path.ctrl, u);


        geometry_msgs::msg::PoseStamped ps;

        ps.header = msg.header;

        double px_m = 0.01 * p.x;

        double py_m = 0.01 * p.y;

        double th_map = th;

        if (to_map)
        {
            // ctrl_x/ctrl_y 를 채울 때와 같은 변환. msg.header.frame_id
            // 가 "map" 인데 base_link 좌표를 그대로 실으면 RViz 에서
            // 위치가 어긋난다.
            const double x_m = px_m;

            const double y_m = py_m;

            px_m = vehicle_pose.x_m + x_m * cos_yaw - y_m * sin_yaw;

            py_m = vehicle_pose.y_m + x_m * sin_yaw + y_m * cos_yaw;

            th_map = th + vehicle_pose.yaw_rad;
        }

        ps.pose.position.x = px_m;

        ps.pose.position.y = py_m;

        ps.pose.position.z = 0.0;

        ps.pose.orientation.z = std::sin(0.5 * th_map);

        ps.pose.orientation.w = std::cos(0.5 * th_map);


        viz.poses.push_back(ps);
    }


    viz_path_publisher_->publish(viz);
}
