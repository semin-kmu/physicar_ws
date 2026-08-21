"""system_supervisor 노드. 기동·감시·복구·System 상태. 설계는 docs/01~04."""

import rclpy
from rclpy.node import Node


class SystemSupervisor(Node):
    """감시 tick 20 Hz. 발행은 tick 안에서만 (00 불변식 1)."""

    def __init__(self) -> None:
        super().__init__('system_supervisor')

        self.declare_parameter('tick_hz', 20.0)
        tick_hz = self.get_parameter('tick_hz').value

        self._timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(f'system_supervisor 기동. tick={tick_hz} Hz')

    def _tick(self) -> None:
        """감시 1주기. 판정 → 상태 갱신 → 발행 순서."""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
