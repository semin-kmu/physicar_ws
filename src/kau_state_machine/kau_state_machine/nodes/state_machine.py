"""state_machine 노드. Mission·Behavior 상태, 진행도, 속도 상한. 설계는 docs/05~06."""

import rclpy
from rclpy.node import Node


class StateMachine(Node):
    """평가 tick 20 Hz. 발행은 tick 안에서만 (00 불변식 1)."""

    def __init__(self) -> None:
        super().__init__('state_machine')

        self.declare_parameter('tick_hz', 20.0)
        tick_hz = self.get_parameter('tick_hz').value

        self._timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info(f'state_machine 기동. tick={tick_hz} Hz')

    def _tick(self) -> None:
        """평가 1주기. 입력 판정 → 상태 전이 → 발행 순서."""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateMachine()
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
