#!/usr/bin/env python3
"""HLS 임계값 수동 튜닝을 위한 스냅샷 저장 노드.

카메라(기본은 BEV) 영상을 구독해 초당 한 장 정도로 디스크에 저장한다.
흰색 차선이 잘 안 잡힐 때, 여러 프레임을 모아 놓고 오프라인에서
(GIMP, python 스크립트 등으로) HLS 값을 직접 재서 임계값을 잡으려는
용도다. 실시간으로 슬라이더를 움직이며 보려면 hls_tuner.py 를 쓴다.

    ros2 run kau_lane_detection hls_snapshot_saver.py
    ros2 run kau_lane_detection hls_snapshot_saver.py --dir ~/snaps --hz 2.0
    ros2 run kau_lane_detection hls_snapshot_saver.py --topic /camera/image_raw

기본 입력은 BEV 영상이다. 노드가 HLS 마스크를 BEV 위에서 만들기 때문에
여기서 저장해야 실제 임계값과 좌표계가 맞는다 (hls_tuner.py 와 동일한
전제). BEV 디버그 이미지는 구독자가 있을 때만 발행되므로, 이 노드가
붙어 있는 동안만 kau_lane_detection_node 가 그 토픽을 낸다.
"""

import argparse
import os
import time
from datetime import datetime

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


DEFAULT_TOPIC = "/kau_lane_detection/bev_image"

DEFAULT_DIR = os.path.expanduser("~/kau_hls_snapshots")

DEFAULT_HZ = 1.0


class HlsSnapshotSaverNode(Node):

    def __init__(self, topic, save_dir, hz):

        super().__init__("kau_hls_snapshot_saver")

        self.bridge = CvBridge()

        self.save_dir = save_dir

        # hz <= 0 이면 매 프레임 저장 (제한 없음).
        self.min_interval = 1.0 / hz if hz > 0 else 0.0

        self.last_saved = 0.0

        self.count = 0

        os.makedirs(self.save_dir, exist_ok=True)

        self.create_subscription(
            Image,
            topic,
            self.on_image,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f"subscribed: {topic} -> {self.save_dir} (~{hz:.2g} Hz)"
        )


    def on_image(self, msg):

        now = time.monotonic()

        if now - self.last_saved < self.min_interval:

            return


        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception as exc:

            self.get_logger().error(f"convert failed: {exc}")

            return


        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        filename = os.path.join(self.save_dir, f"snapshot_{stamp}.jpg")

        # 화질 저하 없이(비압축에 가깝게) 저장 - 나중에 HLS 값을 재는
        # 용도라 JPEG 압축 손실이 임계값 판단을 흔들면 안 된다.
        ok = cv2.imwrite(filename, image, [cv2.IMWRITE_JPEG_QUALITY, 100])

        if not ok:

            self.get_logger().error(f"저장 실패: {filename}")

            return


        self.last_saved = now

        self.count += 1

        self.get_logger().info(f"[{self.count}] saved {filename}")


def main():

    parser = argparse.ArgumentParser(
        description="HLS 튜닝용 스냅샷 저장 노드"
    )

    parser.add_argument(
        "--topic", default=DEFAULT_TOPIC,
        help=f"입력 영상 토픽 (기본 {DEFAULT_TOPIC})"
    )

    parser.add_argument(
        "--dir", default=DEFAULT_DIR,
        help=f"저장 디렉토리 (기본 {DEFAULT_DIR})"
    )

    parser.add_argument(
        "--hz", type=float, default=DEFAULT_HZ,
        help=f"저장 주기 [Hz] (기본 {DEFAULT_HZ})"
    )

    args, _ = parser.parse_known_args()

    rclpy.init()

    node = HlsSnapshotSaverNode(args.topic, args.dir, args.hz)

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
