// ====================================================================
// odom_covariance_relay.cpp
//
// /odom/laser 와 /imu 에 **공분산을 채워서** 다시 낸다. 그것만 한다.
// 값은 하나도 안 건드리고 covariance 배열만 덮어쓴다.
//
// ── 왜 이 노드가 필요한가 ──────────────────────────────────────────
// robot_localization 에는 센서별 신뢰도 파라미터가 없다. odom0_* 로
// 줄 수 있는 것은 _config / _differential / _relative / _queue_size /
// _rejection_threshold 뿐이고, **측정 신뢰도는 오직 메시지의
// covariance 필드에서 온다.**
//
// 그런데 우리 두 소스는 공분산을 아예 안 채운다:
//
//   physicar_laser_odom/src/laser_odom_node.cpp
//       "covariance" 라는 문자열 자체가 없다 -> pose/twist 전부 0
//
//   physicar_bringup/src/physicar_driver_node.cpp:1156-1162
//       linear_acceleration / angular_velocity 만 채우고
//       orientation_covariance[0] = -1.0 만 찍는다
//       -> angular_velocity_covariance / linear_acceleration_covariance
//          전부 0
//
// robot_localization 은 공분산 대각이 1e-9 미만이면 1e-9 로 올려 쓴다.
// 즉 지금은 **라이다도 IMU도 "오차 0 인 완벽한 측정"** 으로 들어간다.
// 칼만 이득이 사실상 1 이라 필터가 갱신마다 마지막에 들어온 측정으로
// 그냥 스냅한다. config/ekf.yaml 주석의 실측이 이걸로 설명된다 —
// 두 소스는 서로 0.99 도로 잘 맞는데 (|/odom/laser - /imu| 평균 0.99)
// EKF 만 혼자 7.9 deg/min 로 틀어지고 주행 중 19 도까지 벌어진다.
// 소스가 나빠서가 아니라 필터가 둘 다 무한 신뢰해서 사이를 튀는 것이다.
//
// 그래서 "라이다 신뢰도를 낮추고 IMU 와 블렌딩" 은 스칼라 하나로 되는
// 일이 아니고, 공분산을 실제로 채워야 비로소 성립한다. 이 노드가
// 그 손잡이다.
//
// ── 왜 /opt/physicar 소스를 안 고치나 ──────────────────────────────
// ekf.launch.py 주석과 같은 이유다. 플랫폼이 초기화되면 같이 날아가고
// (updater.sh), 다른 팀원 환경과도 어긋난다. 원리적으로는 ICP 잔차에
// 비례한 동적 공분산을 laser_odom 안에서 내는 쪽이 낫지만, 대회 전에
// 환경이 갈라지지 않는 것이 우선이다.
//
// ── 한계 ───────────────────────────────────────────────────────────
// 여기서 넣는 것은 **고정 공분산** 이다. ICP 가 실제로 잘 맞은 프레임과
// 미끄러진 프레임을 구별하지 못한다. 그걸 하려면 잔차가 필요하고,
// 잔차는 laser_odom 안에만 있다. 지금은 "평균적으로 이 정도 못 믿는다"
// 를 넣는 것이고, 그것만으로도 무한신뢰보다는 훨씬 낫다.
// ====================================================================

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>


namespace
{

// 6x6 row-major 공분산에서 대각 원소의 인덱스.
// 순서: x, y, z, roll, pitch, yaw  (twist 는 vx, vy, vz, vroll, vpitch, vyaw)
constexpr std::size_t kDiag6[6] = {0, 7, 14, 21, 28, 35};

// 쓰지 않는 축에 넣는 값. robot_localization 은 _config 에서 켠 축만
// 읽으므로 사실 무시되지만, 다른 도구(rviz, rosbag 분석)가 이 메시지를
// 볼 때 "이 축은 근거 없음" 이 드러나도록 크게 박아 둔다.
constexpr double kUnused = 1.0e6;


double variance(double sigma)
{
    return sigma * sigma;
}

}  // namespace


class OdomCovarianceRelay : public rclcpp::Node
{
public:
    OdomCovarianceRelay()
    : rclcpp::Node("odom_covariance_relay")
    {
        // ------------------------------------------------------------
        // 라이다 오도메트리
        //
        // sigma 는 "이 측정이 평균적으로 이만큼 틀린다" 는 표준편차다.
        // 크게 잡을수록 EKF 가 덜 믿는다 = 모션 모델 쪽으로 무게가 간다.
        // ------------------------------------------------------------
        laser_in_ = declare_parameter<std::string>(
            "laser.in_topic", "/odom/laser");

        laser_out_ = declare_parameter<std::string>(
            "laser.out_topic", "/odom/laser_cov");

        laser_sigma_xy_ = declare_parameter<double>(
            "laser.sigma_xy_m", 0.10);

        laser_sigma_yaw_ = declare_parameter<double>(
            "laser.sigma_yaw_rad", 0.05);

        laser_sigma_vx_ = declare_parameter<double>(
            "laser.sigma_vx_ms", 0.20);

        laser_sigma_vyaw_ = declare_parameter<double>(
            "laser.sigma_vyaw_rads", 0.20);


        // ------------------------------------------------------------
        // IMU
        //
        // orientation 은 **건드리지 않는다.** 실기 드라이버가
        // orientation_covariance[0] = -1.0 으로 "이 값 없음" 을 찍어
        // 두는데 그 규약을 지우면 안 된다 (ekf.yaml 의 "왜 IMU 가 아니라
        // 스캔 yaw 인가" 참고). 우리가 채우는 것은 각속도와 가속도뿐.
        // ------------------------------------------------------------
        imu_in_ = declare_parameter<std::string>(
            "imu.in_topic", "/imu");

        imu_out_ = declare_parameter<std::string>(
            "imu.out_topic", "/imu/cov");

        imu_sigma_vyaw_ = declare_parameter<double>(
            "imu.sigma_vyaw_rads", 0.02);

        imu_sigma_accel_ = declare_parameter<double>(
            "imu.sigma_accel_ms2", 0.30);


        // 원본이 이미 공분산을 채워 오면 덮지 않는 안전장치.
        //
        // 기본값 false 다. 실측(2026-08-25)에서 /odom/laser 는 sim·실차
        // 모두 0 인데 **sim IMU 는 Gazebo 가 4e-08 을 채워 준다**
        // (sigma 0.011 deg/s). 그건 정직한 값이라 덮으면 sim 성능만
        // 깎인다. false 면 "0 인 것만 채운다" 가 되어 라이다는 항상
        // 채워지고 IMU 는 sim 에서 원본이 살고 실차에서만 우리 값이
        // 들어간다 -- 한 설정으로 두 환경이 각자 옳게 동작한다.
        overwrite_ = declare_parameter<bool>(
            "overwrite_existing", false);


        // ------------------------------------------------------------
        // QoS
        //
        // 구독은 BEST_EFFORT (SensorDataQoS) 로 받는다 — 발행측이
        // RELIABLE 이든 BEST_EFFORT 든 호환된다 (구독이 덜 요구하는
        // 쪽이라 항상 붙는다).
        //
        // 발행은 RELIABLE 기본 QoS 로 낸다 — robot_localization 이
        // 기본 QoS 로 구독하므로 여기서 BEST_EFFORT 로 내면
        // **호환이 깨져서 EKF 가 아무것도 못 받는다.**
        // ------------------------------------------------------------
        laser_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(laser_out_, 10);

        imu_pub_ =
            create_publisher<sensor_msgs::msg::Imu>(imu_out_, 10);

        laser_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            laser_in_,
            rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                onLaser(msg);
            });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_in_,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                onImu(msg);
            });


        RCLCPP_INFO(
            get_logger(),
            "공분산 릴레이 시작. %s -> %s (sigma xy %.3f m, yaw %.3f rad, "
            "vx %.3f m/s) | %s -> %s (sigma vyaw %.3f rad/s)",
            laser_in_.c_str(), laser_out_.c_str(),
            laser_sigma_xy_, laser_sigma_yaw_, laser_sigma_vx_,
            imu_in_.c_str(), imu_out_.c_str(), imu_sigma_vyaw_);
    }


private:
    // 원본이 이미 의미 있는 값을 갖고 있는가.
    // robot_localization 의 문턱(1e-9)과 같은 기준을 쓴다.
    static bool alreadyFilled(const std::array<double, 36> & cov)
    {
        for (std::size_t i = 0; i < 6; ++i)
        {
            if (std::fabs(cov[kDiag6[i]]) > 1.0e-9)
            {
                return true;
            }
        }

        return false;
    }


    void onLaser(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry out = *msg;

        if (overwrite_ || !alreadyFilled(out.pose.covariance))
        {
            out.pose.covariance.fill(0.0);

            out.pose.covariance[kDiag6[0]] = variance(laser_sigma_xy_);
            out.pose.covariance[kDiag6[1]] = variance(laser_sigma_xy_);
            out.pose.covariance[kDiag6[2]] = kUnused;   // z
            out.pose.covariance[kDiag6[3]] = kUnused;   // roll
            out.pose.covariance[kDiag6[4]] = kUnused;   // pitch
            out.pose.covariance[kDiag6[5]] = variance(laser_sigma_yaw_);
        }

        if (overwrite_ || !alreadyFilled(out.twist.covariance))
        {
            out.twist.covariance.fill(0.0);

            out.twist.covariance[kDiag6[0]] = variance(laser_sigma_vx_);
            out.twist.covariance[kDiag6[1]] = kUnused;  // vy
            out.twist.covariance[kDiag6[2]] = kUnused;  // vz
            out.twist.covariance[kDiag6[3]] = kUnused;  // vroll
            out.twist.covariance[kDiag6[4]] = kUnused;  // vpitch
            out.twist.covariance[kDiag6[5]] = variance(laser_sigma_vyaw_);
        }

        laser_pub_->publish(out);
    }


    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu out = *msg;

        // orientation_covariance 는 그대로 둔다. 실기의 -1.0 규약을
        // 지우면 나중에 누가 imu0_config 의 yaw 를 켤 때 "이 값 없음"
        // 이라는 유일한 표식이 사라진다.

        const bool gyro_filled =
            std::fabs(out.angular_velocity_covariance[8]) > 1.0e-9;

        if (overwrite_ || !gyro_filled)
        {
            out.angular_velocity_covariance.fill(0.0);

            out.angular_velocity_covariance[0] = kUnused;  // vroll
            out.angular_velocity_covariance[4] = kUnused;  // vpitch
            out.angular_velocity_covariance[8] =
                variance(imu_sigma_vyaw_);
        }

        const bool accel_filled =
            std::fabs(out.linear_acceleration_covariance[0]) > 1.0e-9;

        if (overwrite_ || !accel_filled)
        {
            out.linear_acceleration_covariance.fill(0.0);

            out.linear_acceleration_covariance[0] =
                variance(imu_sigma_accel_);
            out.linear_acceleration_covariance[4] =
                variance(imu_sigma_accel_);
            out.linear_acceleration_covariance[8] = kUnused;  // az
        }

        imu_pub_->publish(out);
    }


    std::string laser_in_;
    std::string laser_out_;
    std::string imu_in_;
    std::string imu_out_;

    double laser_sigma_xy_;
    double laser_sigma_yaw_;
    double laser_sigma_vx_;
    double laser_sigma_vyaw_;

    double imu_sigma_vyaw_;
    double imu_sigma_accel_;

    bool overwrite_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomCovarianceRelay>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            node->get_logger(), "치명적 예외로 종료합니다: %s", e.what());
    }

    rclcpp::shutdown();

    return 0;
}
