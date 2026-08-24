"""state_machine 노드. Mission·Behavior 상태, 진행도, 속도 상한. 설계는 docs/05~06."""

import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


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
    """SIGINT/SIGTERM 을 직접 받아 곱게 내려온다.

    rclpy 기본 신호 처리(SignalHandlerOptions.ALL)는 컨텍스트를 **비동기로**
    내린다. rclpy.spin() 은 `while context.ok(): spin_once()` 인데, ok() 를
    통과한 직후 컨텍스트가 내려가면 wait set 생성이 RCLError 로 터진다.
    ExternalShutdownException 이 아니라서 안 잡히고 traceback 으로 끝나며,
    종료 코드도 0 이 아니게 되어 supervisor 의 사망 판정을 오염시킨다
    (실측 2026-08-24: state_machine.out 의 "failed to initialize wait set").

    신호를 직접 받아 루프를 빠져나오면 그 창 자체가 없다.
    platform_ekf_pause.py 와 같은 방식이다.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StateMachine()

    alive = [True]

    def stop(_signum, _frame):
        alive[0] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while alive[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
