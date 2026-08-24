"""system_supervisor 노드. 기동·감시·복구·System 상태. 설계는 docs/01~04.

현 구현 범위는 **순차 spawn + 역순 종료**뿐이다 (docs/01 section 4-1, 11-1).
관문 G1~G4 · 재시도 · 감시 · enable 발행은 미구현.
"""

import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from kau_state_machine.log import err, log
from kau_state_machine.ros.params import load_manifest, make_run_id, resolve_log_dir
from kau_state_machine.ros.process import kill, spawn

PACKAGE = 'kau_state_machine'


class SystemSupervisor(Node):
    """자신을 제외한 전 노드를 자식 프로세스로 소유한다."""

    def __init__(self) -> None:
        super().__init__('system_supervisor')

        default_yaml = os.path.join(
            get_package_share_directory(PACKAGE), 'config', 'bringup.yaml')
        self.declare_parameter('bringup_yaml', default_yaml)
        self.declare_parameter('run_id', '')

        manifest_path = self.get_parameter('bringup_yaml').value
        self._manifest = load_manifest(manifest_path)

        run_id = self.get_parameter('run_id').value or make_run_id()
        self._log_dir = resolve_log_dir(self._manifest.log_dir, run_id)

        self._children = []
        self._lock = threading.Lock()

        log('supervisor', f'manifest={manifest_path}')
        log('supervisor', f'log_dir={self._log_dir}')

        # spawn 은 블로킹이므로 spin 과 분리한다 (01 section 7-2).
        self._thread = threading.Thread(target=self._bringup, daemon=True)
        self._thread.start()

    def _bringup(self) -> None:
        """단계 오름차순. 단계 안은 동시 spawn, 사이는 delay_sec 대기."""
        total = len(self._manifest.stages)
        for index, stage in enumerate(self._manifest.stages, 1):
            log('bringup', f'stage {index}/{total} (id={stage.id})')
            for spec in stage.nodes:
                try:
                    child = spawn(spec, self._log_dir)
                except Exception as exc:  # noqa: BLE001
                    err('bringup', f'중단 · {spec.name} spawn 실패: {exc}')
                    return
                with self._lock:
                    self._children.append(child)
                log('spawn', f'pid={child.pid}', node=spec.name)
            if stage.delay_sec:
                time.sleep(stage.delay_sec)
        log('bringup', f'전 단계 spawn 완료 · {self._manifest.node_count}개')

    def shutdown(self) -> None:
        """기동의 역순으로 정리. 실패해도 강행한다 (01 section 11-2)."""
        with self._lock:
            children, self._children = list(self._children), []
        if not children:
            return

        log('shutdown', '역순 정리 시작')
        for child in reversed(children):
            code = kill(child)
            log('kill', f'pid={child.pid} rc={code}', node=child.name)
        log('shutdown', '완료')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
