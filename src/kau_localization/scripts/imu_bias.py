#!/usr/bin/env python3
"""정지 상태에서 IMU 바이어스/노이즈와 라이다 오도메트리 노이즈를 잰다.

여기서 나오는 숫자가 config/odom_covariance.yaml 의 sigma_* / bias_* 를
정하는 근거다. **차를 세워 두고** 돌려야 한다 (모터도 끄는 게 좋다).

  ros2 run kau_localization imu_bias.py --secs 60

무엇을 재나
  /imu            angular_velocity.z   -> 자이로 z 바이어스 + 노이즈
                  linear_acceleration  -> 가속도 바이어스 + 노이즈
  /odom/laser     twist.angular.z      -> ICP 요레이트 노이즈
                  twist.linear.x       -> ICP 전진속도 노이즈
                  pose (yaw, x, y)     -> 정지 중 누적 드리프트

바이어스 vs 노이즈
  바이어스(평균)는 **빼야 하는 값**이다  -> bias_* 파라미터
  노이즈(표준편차)는 **믿는 정도**다      -> sigma_* 파라미터
  이 둘을 섞으면 안 된다. sigma 를 키워서 바이어스를 가릴 수는 없다.

가속도 바이어스의 주 원인은 센서가 아니라 **장착 기울기**다.
실기 드라이버가 orientation 을 단위 쿼터니언(완전 수평)으로 두기 때문에
robot_localization 이 중력을 수직으로만 빼고, 장착이 δ 도 기울어 있으면
g*sin(δ) 가 그대로 ax/ay 에 남는다 (1 도 = 0.171 m/s^2).
그래서 수평 바이어스를 기울기 각으로 환산해서 같이 보여 준다.

--topic /imu/cov 로 릴레이 출력을 재면 보정이 먹었는지 확인할 수 있다.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

G = 9.80665
TILT_WARN_DEG = 2.0          # 이 이상 기울었으면 장착을 고치는 게 낫다
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stats(v):
    if not v:
        return 0.0, 0.0, 0.0
    n = len(v)
    mean = sum(v) / n
    var = sum((x - mean) ** 2 for x in v) / n if n > 1 else 0.0
    return mean, math.sqrt(var), max(abs(x) for x in v)


class Collector(Node):
    def __init__(self, imu_topic, odom_topic):
        super().__init__('imu_bias')
        self.gz, self.ax, self.ay, self.az = [], [], [], []
        self.lvyaw, self.lvx = [], []
        self.first_pose = None
        self.last_pose = None
        self.first_t = None
        self.last_t = None
        self.cmd = 0

        self.create_subscription(Imu, imu_topic, self.on_imu, SENSOR_QOS)
        self.create_subscription(Odometry, odom_topic, self.on_odom, SENSOR_QOS)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_imu(self, m):
        self.gz.append(m.angular_velocity.z)
        self.ax.append(m.linear_acceleration.x)
        self.ay.append(m.linear_acceleration.y)
        self.az.append(m.linear_acceleration.z)

    def on_odom(self, m):
        self.lvyaw.append(m.twist.twist.angular.z)
        self.lvx.append(m.twist.twist.linear.x)
        p = (m.pose.pose.position.x, m.pose.pose.position.y,
             yaw_of(m.pose.pose.orientation))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.first_pose is None:
            self.first_pose, self.first_t = p, t
        self.last_pose, self.last_t = p, t

    def on_cmd(self, m):
        if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6:
            self.cmd += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--secs', type=float, default=60.0)
    ap.add_argument('--topic', default='/imu', help='IMU 토픽 (/imu/cov 로 보정 확인)')
    ap.add_argument('--odom', default='/odom/laser', help='라이다 오도메트리 토픽')
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.topic, args.odom)
    print(f'{args.secs:.0f} 초 동안 {args.topic} / {args.odom} 를 모은다. '
          f'차를 절대 건드리지 마라.', flush=True)

    end = node.get_clock().now().nanoseconds * 1e-9 + args.secs
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    gz_m, gz_s, _ = stats(node.gz)
    ax_m, ax_s, _ = stats(node.ax)
    ay_m, ay_s, _ = stats(node.ay)
    az_m, az_s, _ = stats(node.az)
    lw_m, lw_s, lw_max = stats(node.lvyaw)
    lx_m, lx_s, lx_max = stats(node.lvx)

    print()
    print(f'=== IMU {args.topic} ({len(node.gz)} 샘플) ===')
    if not node.gz:
        print('  샘플이 없다. 토픽 이름과 QoS 를 확인할 것.')
        rclpy.shutdown()
        return 1
    print(f'  gyro z   평균 {gz_m:+.6f} rad/s  ({math.degrees(gz_m) * 60:+.2f} deg/min)'
          f'   sigma {gz_s:.5f}')
    print(f'  accel x  평균 {ax_m:+.4f} m/s^2   sigma {ax_s:.4f}')
    print(f'  accel y  평균 {ay_m:+.4f} m/s^2   sigma {ay_s:.4f}')
    print(f'  accel z  평균 {az_m:+.4f} m/s^2   sigma {az_s:.4f}'
          f'   (중력 {G:.3f} 대비 {az_m - G:+.4f})')

    horiz = math.hypot(ax_m, ay_m)
    tilt = math.degrees(math.asin(min(1.0, horiz / G)))
    print(f'  수평 바이어스 크기 {horiz:.4f} m/s^2  ->  장착 기울기 {tilt:.2f} 도')
    if tilt > TILT_WARN_DEG:
        print(f'  ★ {TILT_WARN_DEG} 도를 넘는다. 물리적으로 수평을 맞추는 게 우선이다.')
        print(f'    보정만 하면 0.5*b*t^2 로 자라는 위치 오차는 막지만, 바이어스가')
        print(f'    온도/시간에 따라 흔들리면 잔차가 그대로 남는다.')

    print()
    print(f'=== 라이다 {args.odom} ({len(node.lvyaw)} 샘플) ===')
    if node.lvyaw:
        print(f'  twist vyaw  평균 {lw_m:+.5f} rad/s  sigma {lw_s:.5f}  최대 |{lw_max:.5f}|')
        print(f'  twist vx    평균 {lx_m:+.5f} m/s    sigma {lx_s:.5f}  최대 |{lx_max:.5f}|')
    if node.first_pose and node.last_t and node.last_t > node.first_t:
        dt = node.last_t - node.first_t
        dx = node.last_pose[0] - node.first_pose[0]
        dy = node.last_pose[1] - node.first_pose[1]
        dyaw = math.degrees(node.last_pose[2] - node.first_pose[2])
        print(f'  정지 {dt:.1f} 초 누적 드리프트: '
              f'위치 {math.hypot(dx, dy):.3f} m, yaw {dyaw:+.2f} deg '
              f'({dyaw / dt * 60:+.2f} deg/min)')

    if node.cmd:
        print()
        print(f'★ 측정 중 /cmd_vel 이 {node.cmd} 번 왔다. 정지 측정이 아니다. 다시 재라.')

    print()
    print('=== config/odom_covariance.yaml 에 붙일 값 ===')
    print('    imu:')
    print(f'      sigma_vyaw_rads: {max(gz_s, 1e-4):.5f}')
    print(f'      sigma_accel_ms2: {max(ax_s, ay_s, 1e-3):.4f}')
    print(f'      bias_vyaw_rads: {gz_m:.7f}')
    print(f'      bias_ax_ms2: {ax_m:.4f}')
    print(f'      bias_ay_ms2: {ay_m:.4f}')
    if node.lvyaw:
        print('    laser:')
        print(f'      sigma_vyaw_rads: {max(lw_s, 1e-4):.5f}')
        print(f'      sigma_vx_ms: {max(lx_s, 1e-4):.5f}')
    print()
    print('※ sigma 는 정지 노이즈다. 주행 중에는 ICP 가 더 미끄러지므로')
    print('   laser 쪽은 여기서 나온 값보다 넉넉하게(2~5배) 잡는 것이 맞다.')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
