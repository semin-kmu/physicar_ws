#!/usr/bin/env python3
# ============================================================
# /camera/set_camera_info 한 번 불러 주고 끝나는 일회성 노드
#
# 실차 카메라 드라이버(camera_ros/libcamera)는 캘리브 파일이 없으면
# intrinsic 을 **0 으로 채워** 발행한다. 그걸 받으면 인지 노드는
# "CameraInfo 의 intrinsic 이 유효하지 않습니다" 를 띄우고 멈춘다.
#
# 그래서 사람이 매번 손으로 이걸 쳤다:
#
#   ros2 service call /camera/set_camera_info \
#       sensor_msgs/srv/SetCameraInfo "{camera_info: {...}}"
#
# 이 노드가 그 한 줄을 대신한다. lane_detection.launch.py 가 띄우고,
# 서비스에 값을 넣은 뒤 스스로 종료한다.
#
# 시뮬에는 이 서비스가 없다 (gz bridge 가 camera_info 를 이미 제대로
# 낸다). 그때는 wait_s 만큼 기다렸다가 "건너뜀" 한 줄 남기고 정상
# 종료한다 — required:=true 를 준 경우에만 실패로 취급한다.
# ============================================================

import sys

import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo


DEFAULT_SERVICE = '/camera/set_camera_info'

# 서비스가 올라오기를 기다리는 시간. 카메라 드라이버가 이 런치보다
# 늦게 뜨는 경우가 흔해서 넉넉히 준다.
DEFAULT_WAIT_S = 15.0

# 요청을 보낸 뒤 응답을 기다리는 시간.
DEFAULT_CALL_TIMEOUT_S = 10.0


# ------------------------------------------------------------
# yaml -> sensor_msgs/CameraInfo
#
# 형식은 camera_calibration(camera_info_manager) 표준이다.
# ~/.ros/camera_info/*.yaml 과 같은 형식이라 그대로 복사가 된다.
# ------------------------------------------------------------

def _matrix(doc, key, expect_len):
    """rows/cols/data 블록에서 data 를 꺼낸다. 길이까지 확인한다."""

    if key not in doc:
        raise KeyError(f"'{key}' 항목이 없습니다")

    block = doc[key]

    if not isinstance(block, dict) or 'data' not in block:
        raise ValueError(f"'{key}' 는 rows/cols/data 블록이어야 합니다")

    data = [float(v) for v in block['data']]

    if len(data) != expect_len:
        raise ValueError(
            f"'{key}' 의 data 길이가 {len(data)} 입니다 "
            f"({expect_len} 이어야 합니다)"
        )

    return data


def load_camera_info(path):

    with open(path, 'r') as f:
        doc = yaml.safe_load(f)

    if not isinstance(doc, dict):
        raise ValueError('yaml 최상위가 매핑이 아닙니다')

    msg = CameraInfo()

    msg.width = int(doc['image_width'])
    msg.height = int(doc['image_height'])

    msg.distortion_model = str(doc.get('distortion_model', 'plumb_bob'))

    msg.d = [float(v) for v in doc['distortion_coefficients']['data']]

    msg.k = _matrix(doc, 'camera_matrix', 9)
    msg.r = _matrix(doc, 'rectification_matrix', 9)
    msg.p = _matrix(doc, 'projection_matrix', 12)

    # 인지 노드의 유효성 검사와 같은 기준으로 여기서 먼저 막는다.
    # 잘못된 값을 드라이버에 저장해 버리면 (camera_info_manager 는
    # ~/.ros/camera_info/*.yaml 로 **영구 저장**한다) 다음 부팅부터
    # 계속 그 값이 살아난다.
    fx, cx = msg.k[0], msg.k[2]
    fy, cy = msg.k[4], msg.k[5]

    if not (fx > 1.0 and fy > 1.0):
        raise ValueError(f'초점거리가 이상합니다 (fx={fx} fy={fy})')

    if not (0.0 < cx < msg.width and 0.0 < cy < msg.height):
        raise ValueError(
            f'주점이 영상 밖입니다 (cx={cx} cy={cy}, '
            f'영상 {msg.width}x{msg.height})'
        )

    return msg


# ============================================================
# Node
# ============================================================

class SetCameraInfoNode(Node):

    def __init__(self):

        super().__init__('kau_set_camera_info')

        self.camera_info_file = str(
            self.declare_parameter('camera_info_file', '').value
        )

        self.service_name = str(
            self.declare_parameter('service_name', DEFAULT_SERVICE).value
        )

        self.wait_s = float(
            self.declare_parameter('wait_s', DEFAULT_WAIT_S).value
        )

        self.call_timeout_s = float(
            self.declare_parameter(
                'call_timeout_s', DEFAULT_CALL_TIMEOUT_S).value
        )

        # 서비스가 없을 때 실패로 볼 것인가. 시뮬에서는 없는 게 정상이라
        # 기본은 false 다.
        self.required = bool(
            self.declare_parameter('required', False).value
        )

    # --------------------------------------------------------

    def run(self):
        """0 = 성공 또는 '건너뜀', 1 = 실패."""

        log = self.get_logger()

        if not self.camera_info_file:
            log.error(
                'camera_info_file 파라미터가 비어 있습니다. '
                '보낼 캘리브레이션 파일을 지정하십시오.'
            )
            return 1

        try:
            info = load_camera_info(self.camera_info_file)

        except Exception as exc:      # noqa: BLE001 - 원인을 그대로 보여준다
            log.error(
                f'캘리브레이션 파일을 읽지 못했습니다 '
                f'({self.camera_info_file}): {exc}'
            )
            return 1

        client = self.create_client(SetCameraInfo, self.service_name)

        log.info(
            f'{self.service_name} 를 기다립니다 '
            f'(최대 {self.wait_s:.0f}s) ...'
        )

        if not client.wait_for_service(timeout_sec=self.wait_s):

            message = (
                f'{self.service_name} 서비스가 없어 건너뜁니다. '
                f'시뮬(gz bridge 가 camera_info 를 직접 냄)이면 정상입니다. '
                f'실차인데 이 줄이 보이면 카메라 드라이버가 아직 안 떴거나 '
                f'서비스 이름이 다른 것입니다 '
                f'(ros2 service list | grep set_camera_info).'
            )

            if self.required:
                log.error(message)
                return 1

            log.warning(message)
            return 0

        request = SetCameraInfo.Request()
        request.camera_info = info

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.call_timeout_s)

        if not future.done():
            log.error(
                f'{self.service_name} 응답이 '
                f'{self.call_timeout_s:.0f}s 안에 오지 않았습니다.'
            )
            return 1

        response = future.result()

        if response is None or not response.success:

            reason = '' if response is None else response.status_message

            log.error(
                f'{self.service_name} 호출이 실패했습니다: {reason}'
            )
            return 1

        log.info(
            f'CameraInfo 적용 완료 — {info.width}x{info.height}, '
            f'fx={info.k[0]:.3f} fy={info.k[4]:.3f} '
            f'cx={info.k[2]:.3f} cy={info.k[5]:.3f}, '
            f'D={[round(v, 6) for v in info.d]} '
            f'({self.camera_info_file})'
        )

        if response.status_message:
            log.info(f'드라이버 응답: {response.status_message}')

        return 0


# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SetCameraInfoNode()

    try:
        code = node.run()

    except KeyboardInterrupt:
        code = 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
