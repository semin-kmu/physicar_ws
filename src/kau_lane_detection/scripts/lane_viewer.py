#!/usr/bin/env python3

import os
import signal
import threading
import time
import socket
import logging

import cv2

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from flask import Flask, Response, jsonify
from werkzeug.serving import make_server


PORT = 5000


# ============================================================
# ROS Lane Viewer Node
# ============================================================

class LaneViewerNode(Node):

    def __init__(self):

        super().__init__(
            "kau_lane_detection_viewer"
        )

        self.bridge = CvBridge()

        # --------------------------------------------------------
        # Latest frame
        # --------------------------------------------------------

        self.latest_frame = None

        self.frame_lock = threading.Lock()

        # --------------------------------------------------------
        # FPS
        # --------------------------------------------------------

        self.frame_count = 0

        self.fps = 0.0

        self.fps_start = time.time()

        # --------------------------------------------------------
        # Status
        # --------------------------------------------------------

        self.status = "..."

        # --------------------------------------------------------
        # ROS subscription
        # --------------------------------------------------------

        self.subscription = self.create_subscription(
            Image,
            "/kau_lane_detection/debug_image",
            self.image_callback,
            10
        )

        self.get_logger().info(
            "Subscribed to "
            "/kau_lane_detection/debug_image"
        )


    # ============================================================
    # Image callback
    # ============================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            # ----------------------------------------------------
            # 최신 프레임 저장
            # ----------------------------------------------------

            with self.frame_lock:

                self.latest_frame = frame.copy()


            # ----------------------------------------------------
            # FPS
            # ----------------------------------------------------

            self.frame_count += 1

            elapsed = (
                time.time()
                - self.fps_start
            )


            if elapsed >= 1.0:

                self.fps = (
                    self.frame_count
                    / elapsed
                )

                self.frame_count = 0

                self.fps_start = time.time()


            # ----------------------------------------------------
            # Status
            # ----------------------------------------------------

            self.status = (
                f"{self.fps:.1f} FPS"
            )


        except Exception as e:

            self.get_logger().error(
                f"Image conversion error: {e}"
            )


    # ============================================================
    # Get latest frame
    # ============================================================

    def get_frame(self):

        with self.frame_lock:

            if self.latest_frame is None:

                return None

            return self.latest_frame.copy()


# ================================================================
# Global
# ================================================================

viewer_node = None

server = None


# ================================================================
# Web Server
# ================================================================

def stop_view():

    global server

    if server is not None:

        server.shutdown()

        server = None


def start_server(app):

    global server

    logging.getLogger(
        "werkzeug"
    ).setLevel(
        logging.ERROR
    )


    # ------------------------------------------------------------
    # Port check
    # ------------------------------------------------------------

    try:

        probe = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        probe.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        probe.bind(
            ("", PORT)
        )

        probe.close()


    except OSError:

        print(
            "web view: port 5000 is already "
            "in use."
        )

        return None


    # ------------------------------------------------------------
    # Create server
    # ------------------------------------------------------------

    server = make_server(
        "0.0.0.0",
        PORT,
        app,
        threaded=True
    )


    # ------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------

    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True
    )

    thread.start()


    print(
        "web view: open the MYAPP tab to watch"
    )


    return server


# ================================================================
# webui.html 경로 탐색
# ================================================================

def find_webui_path():

    candidates = []


    # ------------------------------------------------------------
    # 1. 설치된 share 디렉터리
    # ------------------------------------------------------------

    try:

        from ament_index_python.packages import (
            get_package_share_directory
        )

        candidates.append(
            os.path.join(
                get_package_share_directory(
                    "kau_lane_detection"
                ),
                "web",
                "webui.html"
            )
        )


    except Exception:

        pass


    # ------------------------------------------------------------
    # 2. 소스트리 (colcon build 없이 직접 실행할 때)
    # ------------------------------------------------------------

    candidates.append(
        os.path.join(
            os.path.dirname(
                os.path.dirname(
                    os.path.abspath(__file__)
                )
            ),
            "web",
            "webui.html"
        )
    )


    for path in candidates:

        if os.path.isfile(path):

            return path


    raise FileNotFoundError(
        "webui.html not found. tried: "
        + ", ".join(candidates)
    )


# ================================================================
# Flask App
# ================================================================

def create_app():

    app = Flask(__name__)


    # ============================================================
    # Main page
    #
    # 기존 Line Tracing의 webui.html과 동일한 구조
    #
    # launch 로 실행하면 install 공간에서 돌기 때문에
    # 소스트리 절대경로를 박아두면 파일을 못 찾는다.
    # ament share 디렉터리를 먼저 보고, 없으면 소스트리로 폴백.
    # ============================================================

    page = open(
        find_webui_path()
    ).read()


    # ============================================================
    # /
    # ============================================================

    @app.get("/")
    def index():
        return page


    # ============================================================
    # /frame
    #
    # 기존 Line Tracing의 /frame과 동일한 역할
    # ============================================================

    @app.get("/frame")
    def frame():

        if viewer_node is None:

            return Response(
                status=503
            )


        image = viewer_node.get_frame()


        if image is None:

            return Response(
                status=503
            )


        # --------------------------------------------------------
        # JPEG Encoding
        # --------------------------------------------------------

        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                90
            ]
        )


        if not success:

            return Response(
                status=500
            )


        return Response(
            encoded.tobytes(),
            mimetype="image/jpeg",
            headers={
                "Cache-Control":
                    "no-store"
            }
        )


    # ============================================================
    # /data
    #
    # 기존 Line Tracing의 /data와 동일한 역할
    # ============================================================

    @app.get("/data")
    def data():

        if viewer_node is None:

            return jsonify({
                "status":
                    "viewer not ready"
            })


        return jsonify({
            "status":
                viewer_node.status
        })


    return app


# ================================================================
# Main
# ================================================================

def main(args=None):

    global viewer_node


    # ------------------------------------------------------------
    # ROS2
    # ------------------------------------------------------------

    # SIGINT/SIGTERM 을 직접 받는다.
    #
    # rclpy 기본 신호 처리(SignalHandlerOptions.ALL)는 컨텍스트를 **비동기로**
    # 내린다. 그러면 아래 두 가지가 연달아 터진다.
    #   1. rclpy.spin() 이 ExternalShutdownException 을 던진다 (여기서는
    #      KeyboardInterrupt 만 잡고 있어서 그대로 빠져나간다)
    #   2. finally 의 rclpy.shutdown() 이 이미 내려간 컨텍스트를 또 내리려다
    #      RCLError("rcl_shutdown already called") 로 터진다
    #
    # 그래서 종료 코드가 1 이 되고, kau_state_machine supervisor 의 사망
    # 판정이 오염된다 (실측 2026-08-24: 종료 시 rc=1).
    #
    # 신호를 직접 받아 루프를 빠져나오면 그 창 자체가 없다. 웹서버 스레드는
    # daemon 이라 알아서 죽지만, 포트를 바로 놓아주도록 stop_view() 로
    # 명시해 내린다 (다시 띄울 때 "port 5000 is already in use" 방지).
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )


    viewer_node = LaneViewerNode()


    # ------------------------------------------------------------
    # Flask
    # ------------------------------------------------------------

    app =create_app()


    web_server = start_server(app)


    if web_server is None:

        viewer_node.destroy_node()

        rclpy.try_shutdown()

        return


    alive = [True]


    def stop(_signum, _frame):

        alive[0] = False


    signal.signal(
        signal.SIGINT,
        stop
    )

    signal.signal(
        signal.SIGTERM,
        stop
    )


    try:

        # --------------------------------------------------------
        # ROS spin
        # --------------------------------------------------------

        while alive[0] and rclpy.ok():

            rclpy.spin_once(
                viewer_node,
                timeout_sec=0.1
            )


    finally:

        # --------------------------------------------------------
        # Web server 종료
        # --------------------------------------------------------

        stop_view()


        viewer_node.destroy_node()


        rclpy.try_shutdown()


# ================================================================
# Entry Point
# ================================================================

if __name__ == "__main__":

    main()