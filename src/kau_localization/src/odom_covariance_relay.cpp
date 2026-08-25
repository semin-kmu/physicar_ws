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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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


// 이보다 긴 간격은 적분하지 않는다. IMU 는 50 Hz 이므로 0.5 초면
// 스무 걸음을 놓친 것이고, 그 구멍을 하나의 각속도로 메우면 틀린다.
constexpr double kMaxIntegrationDt = 0.5;


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

        // 횡속도. laser_odom_node.cpp 는 twist.linear.y 를 **한 번도
        // 안 쓴다** (publish_odom 이 linear.x 와 angular.z 만 채운다).
        // 그래서 이 축은 항상 정확히 0 이고, 그 0 을 EKF 에 넣으면
        // "이 차는 옆으로 안 미끄러진다" 는 비홀로노믹 구속이 된다.
        // 애커만 조향차에서 이건 참이고, ax/ay 바이어스가 vy 로
        // 적분돼 쌓이는 것을 막는 유일한 장치다 (ekf.yaml 참고).
        laser_sigma_vy_ = declare_parameter<double>(
            "laser.sigma_vy_ms", 0.05);


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

        // ------------------------------------------------------------
        // IMU 바이어스 (평균 오프셋) — sigma 와 완전히 다른 것이다
        //
        // sigma 는 "얼마나 흔들리나"(노이즈), bias 는 "얼마나 치우쳤나"
        // (평균)다. **sigma 를 키워서 bias 를 가릴 수 없다.** 필터는
        // 치우친 측정을 그냥 느리게 믿을 뿐 결국 따라간다.
        //
        // 가속도 바이어스의 주 원인은 센서가 아니라 **장착 기울기**다.
        // 실기 드라이버가 orientation 을 단위 쿼터니언(완전 수평)으로
        // 두기 때문에 robot_localization 의 중력 제거가 z 에서만 g 를
        // 빼고, 장착이 δ 도 기울면 g*sin(δ) 가 ax/ay 에 그대로 남는다
        // (1 도 = 0.171 m/s^2). 두 번 적분되므로 상수 바이어스 b 는
        // t 초 뒤 0.5*b*t^2 의 위치 오차가 된다.
        //
        // 값은 scripts/imu_bias.py 가 정지 상태에서 재 준다:
        //   ros2 run kau_localization imu_bias.py --secs 60
        //
        // ★ az 는 일부러 안 뺀다. two_d_mode 가 az 상태를 0 으로
        //   고정해서 애초에 읽히지 않는다 (ekf.yaml 참고).
        // ------------------------------------------------------------
        imu_bias_ax_ = declare_parameter<double>(
            "imu.bias_ax_ms2", 0.0);

        imu_bias_ay_ = declare_parameter<double>(
            "imu.bias_ay_ms2", 0.0);

        imu_bias_vyaw_ = declare_parameter<double>(
            "imu.bias_vyaw_rads", 0.0);


        // ------------------------------------------------------------
        // 자이로 적분 방위 (yaw_source)
        //
        // 실기 IMU 에는 orientation 이 없다 (드라이버가 단위 쿼터니언 +
        // orientation_covariance[0] = -1.0 만 찍는다). 그래서 EKF 에
        // **절대 yaw 측정이 하나도 없는 상태**가 되는데, 그러면 yaw
        // 공분산이 무한정 커지고 (실측 4.4 rad^2 = sigma 121 도),
        // 커진 yaw 공분산이 라이다 **위치** 갱신과 교차공분산을 만들어
        // yaw 를 제멋대로 끌고 다닌다.
        //
        //   2026-08-26 실측 (정지 60 초, 절대 yaw 없이 rate 만 융합):
        //     EKF vyaw    -0.553 deg/min   <- 자이로와 일치. 융합은 정상
        //     EKF yaw 각  +6.671 deg/min   <- 자기 각속도와 부호도 반대
        //     EKF yaw 공분산 4.445 rad^2 로 발산
        //
        // 즉 "절대 yaw 를 끄고 rate 로만 섞는다" 는 성립하지 않는다.
        // 절대 yaw 측정은 반드시 하나 있어야 하고, 문제는 그것을
        // **어디서 받느냐**다. 라이다 ICP yaw 는 장면에 따라
        // -33 ~ +19 deg/min 로 흔들리고, 자이로 적분은 -0.02 deg/min 다.
        //
        // 그래서 여기서 자이로 z 를 적분해 orientation 을 만들어 낸다.
        // ekf.yaml 의 imu0_config 가 그 yaw 를 절대 관측으로 읽는다.
        //
        //   auto        : orientation_covariance[0] < 0 ("값 없음" 규약)
        //                 이면 적분해서 채우고, 아니면 원본을 살린다.
        //                 -> 실차는 적분, sim 은 Gazebo 참값 그대로.
        //   integrate   : 항상 적분해서 덮어쓴다
        //   passthrough : 절대 안 건드린다 (예전 동작)
        //
        // ★ 적분값의 원점은 "이 노드가 시작한 순간의 방위" 다. EKF 도
        //   같은 순간에 0 에서 시작하므로 둘이 맞는다. 주행 중에
        //   /set_pose 로 EKF 를 되돌리면 이 적분값도 같이 되돌려야
        //   해서 아래에서 /set_pose 를 구독한다.
        // ------------------------------------------------------------
        imu_yaw_source_ = declare_parameter<std::string>(
            "imu.yaw_source", "auto");

        imu_sigma_yaw_ = declare_parameter<double>(
            "imu.sigma_yaw_rad", 0.02);


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

        // /set_pose 로 EKF 를 되돌리면 적분 방위도 같이 되돌린다.
        // 안 그러면 EKF 만 0 이 되고 IMU 는 옛 방위를 계속 주장해서
        // 필터가 곧바로 되돌아간다 (launch/odom_reset.py 참고).
        // /set_pose 는 노드 이름이 안 붙는 글로벌 이름이다.
        set_pose_sub_ =
            create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/set_pose",
                10,
                [this](
                    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                        msg)
                {
                    const auto & q = msg->pose.pose.orientation;
                    imu_yaw_ = std::atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

                    RCLCPP_INFO(
                        get_logger(),
                        "/set_pose 를 받아 적분 방위를 %.3f rad 로 맞췄다.",
                        imu_yaw_);
                });


        RCLCPP_INFO(
            get_logger(),
            "공분산 릴레이 시작.\n"
            "  %s -> %s  sigma: xy %.3f m, yaw %.3f rad, "
            "vx %.3f m/s, vy %.3f m/s, vyaw %.3f rad/s\n"
            "  %s -> %s  sigma: vyaw %.4f rad/s, accel %.3f m/s^2"
            "  bias: ax %+.4f, ay %+.4f, vyaw %+.6f",
            laser_in_.c_str(), laser_out_.c_str(),
            laser_sigma_xy_, laser_sigma_yaw_, laser_sigma_vx_,
            laser_sigma_vy_, laser_sigma_vyaw_,
            imu_in_.c_str(), imu_out_.c_str(), imu_sigma_vyaw_,
            imu_sigma_accel_, imu_bias_ax_, imu_bias_ay_, imu_bias_vyaw_);
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
            // vy 는 kUnused 가 아니라 진짜 값을 채운다 — 위 선언부
            // 설명대로 이 0 은 "값 없음" 이 아니라 비홀로노믹 구속이다.
            out.twist.covariance[kDiag6[1]] = variance(laser_sigma_vy_);
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

        // 바이어스 제거. 공분산과 달리 이건 **측정값 자체를 고친다.**
        // 0.0 이 기본이라 값을 안 넣으면 아무 일도 일어나지 않는다.
        out.linear_acceleration.x -= imu_bias_ax_;
        out.linear_acceleration.y -= imu_bias_ay_;
        out.angular_velocity.z    -= imu_bias_vyaw_;

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

        // ── 자이로 z 적분 -> 절대 방위 ──
        //
        // 바이어스를 뺀 뒤의 값을 적분한다 (위에서 이미 뺐다).
        // dt 는 메시지 타임스탬프에서 얻는다 -- sim/실기 모두 옳고,
        // 노드가 잠깐 밀려도 적분이 틀어지지 않는다.
        const rclcpp::Time stamp(out.header.stamp);

        if (have_prev_imu_)
        {
            const double dt = (stamp - prev_imu_stamp_).seconds();

            // dt 가 음수면 시간이 뒤로 갔다는 뜻이고 (bag 되감기,
            // sim 리셋), 너무 크면 중간이 비었다는 뜻이다. 둘 다
            // 적분하지 않고 건너뛴다 -- 적분은 틀린 한 걸음이
            // 영원히 남는다.
            if (dt > 0.0 && dt < kMaxIntegrationDt)
            {
                imu_yaw_ += out.angular_velocity.z * dt;
                imu_yaw_ = std::remainder(imu_yaw_, 2.0 * M_PI);
            }
        }

        prev_imu_stamp_ = stamp;
        have_prev_imu_ = true;

        if (integrateYaw(out))
        {
            // roll/pitch 는 0 으로 둔다. two_d_mode 가 어차피 0 으로
            // 고정하고, 실기 IMU 로는 중력 기준 기울기를 안정적으로
            // 낼 수 없다.
            out.orientation.x = 0.0;
            out.orientation.y = 0.0;
            out.orientation.z = std::sin(imu_yaw_ * 0.5);
            out.orientation.w = std::cos(imu_yaw_ * 0.5);

            out.orientation_covariance.fill(0.0);
            out.orientation_covariance[0] = kUnused;  // roll
            out.orientation_covariance[4] = kUnused;  // pitch
            out.orientation_covariance[8] = variance(imu_sigma_yaw_);
        }

        imu_pub_->publish(out);
    }


    // 이 메시지의 orientation 을 우리가 만들어 넣을 것인가.
    bool integrateYaw(const sensor_msgs::msg::Imu & msg) const
    {
        if (imu_yaw_source_ == "passthrough")
        {
            return false;
        }

        if (imu_yaw_source_ == "integrate")
        {
            return true;
        }

        // auto: "값 없음"(-1) 규약이 찍혀 있을 때만 우리가 만든다.
        // 실기 드라이버는 -1 을 찍고, Gazebo 는 참값을 주므로
        // 한 설정으로 두 환경이 각자 옳게 동작한다.
        return msg.orientation_covariance[0] < 0.0;
    }


    std::string laser_in_;
    std::string laser_out_;
    std::string imu_in_;
    std::string imu_out_;

    double laser_sigma_xy_;
    double laser_sigma_yaw_;
    double laser_sigma_vx_;
    double laser_sigma_vyaw_;
    double laser_sigma_vy_;

    double imu_sigma_vyaw_;
    double imu_sigma_accel_;

    double imu_bias_ax_;
    double imu_bias_ay_;
    double imu_bias_vyaw_;

    std::string imu_yaw_source_;
    double imu_sigma_yaw_;

    double imu_yaw_ = 0.0;
    rclcpp::Time prev_imu_stamp_;
    bool have_prev_imu_ = false;

    bool overwrite_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_sub_;
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
