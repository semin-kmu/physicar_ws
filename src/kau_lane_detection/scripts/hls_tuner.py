#!/usr/bin/env python3
"""HLS 범위 측정/튜닝 도구.

시뮬레이터에서 오는 영상을 실시간으로 받아, 슬라이더로 HLS 범위를
조절하면 마스크가 어떻게 변하는지 바로 보여준다.

두 가지 모드:

    ros2 run kau_lane_detection hls_tuner.py            # 웹 (기본, 포트 5001)
    ros2 run kau_lane_detection hls_tuner.py --gui      # OpenCV 창 (noVNC :6080)

웹 모드에만 있는 기능:

    - 영상을 클릭하면 그 지점의 HLS 값을 읽는다(주변 5x5 중앙값).
    - 클릭한 샘플이 쌓이고, 그 min/max 로 범위를 자동 제안한다.
      "잔디를 여러 번 클릭 -> 잔디 범위 확인 -> 그 밖으로 임계 설정"
      이 측정 워크플로의 핵심이다.
    - 현재 값을 실행 중인 노드에 바로 적용(rebuild 불필요).
    - lane_detection.yaml 에 붙여넣을 수 있는 스니펫 출력.

기본 입력은 BEV 영상이다. 노드가 마스크를 BEV 위에서 만들기 때문에
여기서 재야 값이 그대로 맞는다.
"""

import argparse
import logging
import socket
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server


DEFAULT_PORT = 5001

DEFAULT_TOPIC = "/kau_lane_detection/bev_image"

DEFAULT_NODE = "/kau_lane_detection_node"


# 노드의 현재 기본값과 동일하게 맞춰 둔다.
# (kau_lane_detection_node.cpp / config/lane_detection.yaml)
PRESETS = {
    "yellow": {
        "lo": [15, 80, 150],
        "hi": [35, 255, 255],
    },
    "white": {
        "lo": [0, 200, 0],
        "hi": [180, 255, 70],
    },
}

CHANNELS = ("H", "L", "S")

CHANNEL_MAX = (179, 255, 255)


# ================================================================
# ROS Node
# ================================================================

class HlsTunerNode(Node):

    def __init__(self, topic, target_node):

        super().__init__("kau_hls_tuner")

        self.bridge = CvBridge()

        self.frame = None

        self.lock = threading.Lock()

        self.create_subscription(
            Image,
            topic,
            self.on_image,
            10
        )

        self.param_client = self.create_client(
            SetParameters,
            f"{target_node}/set_parameters"
        )

        self.get_logger().info(
            f"subscribed: {topic}"
        )


    def on_image(self, msg):

        try:

            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception as exc:

            self.get_logger().error(
                f"convert failed: {exc}"
            )

            return


        with self.lock:

            self.frame = image


    def get_frame(self):

        with self.lock:

            return None if self.frame is None else self.frame.copy()


    def push_params(self, name, lo, hi):
        """실행 중인 노드에 임계값을 적용한다."""

        if not self.param_client.wait_for_service(timeout_sec=1.0):

            return False, "노드의 set_parameters 서비스에 연결 실패"


        def make(pname, values):

            return Parameter(
                name=pname,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER_ARRAY,
                    integer_array_value=[int(v) for v in values]
                )
            )


        request_msg = SetParameters.Request()

        request_msg.parameters = [
            make(f"{name}_hls_lo", lo),
            make(f"{name}_hls_hi", hi),
        ]

        future = self.param_client.call_async(request_msg)


        # 여기서 spin 하면 안 된다.
        #
        # 웹 모드는 메인 스레드가 rclpy.spin(node) 로 이미 돌고 있어서
        # Flask 스레드가 spin_until_future_complete 를 부르면
        # "Executor is already spinning" 으로 죽는다.
        # 응답은 메인 spin 이 처리하므로 여기서는 완료만 기다린다.

        deadline = time.monotonic() + 3.0


        while not future.done() and time.monotonic() < deadline:

            time.sleep(0.02)


        if not future.done() or future.result() is None:

            return False, "응답 없음(timeout)"


        bad = [
            r.reason
            for r in future.result().results
            if not r.successful
        ]

        if bad:

            return False, "; ".join(bad)


        return True, f"{name}_hls_lo/hi 적용됨"


# ================================================================
# 공유 상태
# ================================================================

class TunerState:

    def __init__(self, target="yellow"):

        self.target = target

        self.lo = list(PRESETS[target]["lo"])

        self.hi = list(PRESETS[target]["hi"])

        self.samples = []          # 클릭으로 모은 HLS 값

        self.lock = threading.Lock()


    def select(self, target):

        with self.lock:

            self.target = target

            self.lo = list(PRESETS[target]["lo"])

            self.hi = list(PRESETS[target]["hi"])


    def snapshot(self):

        with self.lock:

            return self.target, list(self.lo), list(self.hi)


    def add_sample(self, hls):

        with self.lock:

            self.samples.append([int(v) for v in hls])


    def sample_stats(self):
        """모은 샘플의 채널별 min/max 와, 그로부터 제안하는 범위."""

        with self.lock:

            if not self.samples:

                return None

            arr = np.array(self.samples)


        lo = arr.min(axis=0)

        hi = arr.max(axis=0)

        return {
            "n": int(arr.shape[0]),
            "min": lo.tolist(),
            "max": hi.tolist(),
            # 여유 10 을 붙여 제안(경계에 딱 붙이면 프레임마다 깜빡인다)
            "suggest_lo": [
                int(max(0, lo[i] - 10)) for i in range(3)
            ],
            "suggest_hi": [
                int(min(CHANNEL_MAX[i], hi[i] + 10)) for i in range(3)
            ],
        }


# ================================================================
# 마스크 생성
# ================================================================

def build_views(frame, lo, hi):
    """(오버레이, 마스크, 픽셀수) 반환."""

    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    mask = cv2.inRange(
        hls,
        np.array(lo, np.uint8),
        np.array(hi, np.uint8)
    )

    overlay = frame.copy()

    overlay[mask > 0] = (0, 255, 0)

    blended = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0.0)

    return blended, mask, int(cv2.countNonZero(mask))


def sample_hls(frame, x, y, half=2):
    """(x,y) 주변 (2*half+1)^2 패치의 채널별 중앙값."""

    h, w = frame.shape[:2]

    x = int(np.clip(x, 0, w - 1))

    y = int(np.clip(y, 0, h - 1))

    x0, x1 = max(0, x - half), min(w, x + half + 1)

    y0, y1 = max(0, y - half), min(h, y + half + 1)

    hls = cv2.cvtColor(
        frame[y0:y1, x0:x1],
        cv2.COLOR_BGR2HLS
    )

    return np.median(
        hls.reshape(-1, 3),
        axis=0
    ).astype(int).tolist()


def yaml_snippet(target, lo, hi):

    return (
        f"    {target}_hls_lo: [{lo[0]}, {lo[1]}, {lo[2]}]\n"
        f"    {target}_hls_hi: [{hi[0]}, {hi[1]}, {hi[2]}]"
    )


# ================================================================
# Web UI
# ================================================================

PAGE = """<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HLS tuner</title>
<style>
  body { margin:0; background:#14161a; color:#ddd;
         font:13px/1.5 system-ui,sans-serif; }
  #bar { padding:6px 10px; background:#1c1f26; display:flex;
         gap:14px; align-items:center; flex-wrap:wrap; }
  #views { display:flex; gap:6px; padding:6px; flex-wrap:wrap; }
  .pane { flex:1 1 320px; min-width:280px; }
  .pane h4 { margin:0 0 3px; font-size:11px; color:#8a94a6;
             font-weight:600; text-transform:uppercase; }
  img { width:100%; display:block; background:#000; cursor:crosshair; }
  #sliders { display:grid; grid-template-columns:repeat(auto-fit,minmax(230px,1fr));
             gap:4px 16px; padding:6px 10px; }
  .row { display:flex; align-items:center; gap:8px; }
  .row label { width:34px; color:#8a94a6; font-size:11px; }
  .row input[type=range] { flex:1; }
  .row output { width:34px; text-align:right;
                font-variant-numeric:tabular-nums; }
  button { background:#2a2f3a; color:#ddd; border:1px solid #3a4150;
           border-radius:4px; padding:4px 10px; cursor:pointer; font-size:12px; }
  button:hover { background:#333a47; }
  select { background:#2a2f3a; color:#ddd; border:1px solid #3a4150;
           border-radius:4px; padding:3px 6px; }
  pre { margin:0; padding:8px 10px; background:#0f1115; color:#9ecbff;
        font-size:12px; overflow-x:auto; }
  #stats { padding:6px 10px; color:#c9d1d9; }
  .hint { color:#6e7681; font-size:11px; }
  .ok { color:#7ee787; } .err { color:#ff7b72; }
</style>

<div id="bar">
  <select id="target">
    <option value="yellow">yellow</option>
    <option value="white">white</option>
  </select>
  <span id="count" class="hint"></span>
  <button onclick="applySuggest()">샘플 범위로 설정</button>
  <button onclick="clearSamples()">샘플 비우기</button>
  <button onclick="pushNode()">노드에 적용</button>
  <button onclick="resetPreset()">기본값</button>
  <span id="msg" class="hint"></span>
</div>

<div id="views">
  <div class="pane">
    <h4>source + mask (클릭해서 HLS 측정)</h4>
    <img id="src" onclick="sample(event)">
  </div>
  <div class="pane">
    <h4>mask</h4>
    <img id="mask">
  </div>
</div>

<div id="sliders"></div>
<div id="stats"></div>
<pre id="yaml"></pre>

<script>
var CH = ['H','L','S'], MAX = [179,255,255];
var lo = [0,0,0], hi = [0,0,0], busy = false;

function mkRow(kind, i) {
  var d = document.createElement('div'); d.className = 'row';
  d.innerHTML = '<label>' + kind + ' ' + CH[i] + '</label>' +
    '<input type=range min=0 max=' + MAX[i] + ' id="' + kind + i + '">' +
    '<output id="o' + kind + i + '"></output>';
  return d;
}
(function build() {
  var g = document.getElementById('sliders');
  for (var i = 0; i < 3; i++) { g.appendChild(mkRow('lo', i)); g.appendChild(mkRow('hi', i)); }
  for (var i = 0; i < 3; i++) {
    ['lo','hi'].forEach(function (k) {
      document.getElementById(k + i).addEventListener('input', function () { send(); });
    });
  }
})();

function paint() {
  for (var i = 0; i < 3; i++) {
    document.getElementById('lo' + i).value = lo[i];
    document.getElementById('hi' + i).value = hi[i];
    document.getElementById('olo' + i).textContent = lo[i];
    document.getElementById('ohi' + i).textContent = hi[i];
  }
  var t = document.getElementById('target').value;
  document.getElementById('yaml').textContent =
    '    ' + t + '_hls_lo: [' + lo.join(', ') + ']\\n' +
    '    ' + t + '_hls_hi: [' + hi.join(', ') + ']';
}

function readSliders() {
  for (var i = 0; i < 3; i++) {
    lo[i] = +document.getElementById('lo' + i).value;
    hi[i] = +document.getElementById('hi' + i).value;
  }
}

function send() {
  readSliders(); paint();
  fetch('set', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ target: document.getElementById('target').value, lo: lo, hi: hi }) });
}

document.getElementById('target').addEventListener('change', function () {
  fetch('select?target=' + this.value).then(pull);
});

function sample(e) {
  var im = e.target;
  var x = Math.round(e.offsetX / im.clientWidth * im.naturalWidth);
  var y = Math.round(e.offsetY / im.clientHeight * im.naturalHeight);
  fetch('sample?x=' + x + '&y=' + y).then(function (r) { return r.json(); })
    .then(function (d) {
      note('클릭 (' + x + ',' + y + ') -> H' + d.hls[0] + ' L' + d.hls[1] + ' S' + d.hls[2]);
      refreshStats();
    });
}

function applySuggest() { fetch('suggest').then(pull); }
function clearSamples() { fetch('clear').then(function(){ pull(); refreshStats(); }); }
function resetPreset() { fetch('select?target=' + document.getElementById('target').value).then(pull); }
function pushNode() {
  fetch('apply').then(function (r) { return r.json(); }).then(function (d) {
    note(d.message, d.ok ? 'ok' : 'err');
  });
}
function note(t, cls) {
  var m = document.getElementById('msg');
  m.textContent = t; m.className = cls || 'hint';
}

function pull() {
  return fetch('state').then(function (r) { return r.json(); }).then(function (d) {
    lo = d.lo; hi = d.hi;
    document.getElementById('target').value = d.target;
    paint();
  });
}

function refreshStats() {
  fetch('stats').then(function (r) { return r.json(); }).then(function (d) {
    document.getElementById('count').textContent =
      'mask ' + d.pixels + 'px';
    var s = d.samples;
    if (!s) { document.getElementById('stats').innerHTML =
      '<span class="hint">영상을 클릭해 샘플을 모으세요. ' +
      '재려는 대상(예: 잔디, 노란 선)을 여러 군데 찍으면 그 색의 실제 범위가 나옵니다.</span>'; return; }
    document.getElementById('stats').innerHTML =
      '샘플 <b>' + s.n + '</b>개 &nbsp; 실측범위 ' +
      'H ' + s.min[0] + '~' + s.max[0] + ' &nbsp; L ' + s.min[1] + '~' + s.max[1] +
      ' &nbsp; S ' + s.min[2] + '~' + s.max[2] +
      '<br><span class="hint">제안 (여유 ±10): lo [' + s.suggest_lo.join(', ') +
      '] / hi [' + s.suggest_hi.join(', ') + ']</span>';
  });
}

(function loop(id, url) {
  var im = new Image();
  im.onload = function () { document.getElementById(id).src = im.src; setTimeout(function(){loop(id,url);}, 120); };
  im.onerror = function () { setTimeout(function(){loop(id,url);}, 500); };
  im.src = url + '?_=' + Date.now();
})('src', 'frame');
(function loop2() {
  var im = new Image();
  im.onload = function () { document.getElementById('mask').src = im.src; setTimeout(loop2, 120); };
  im.onerror = function () { setTimeout(loop2, 500); };
  im.src = 'maskframe?_=' + Date.now();
})();

pull().then(refreshStats);
setInterval(refreshStats, 1000);
</script>
"""


def jpeg(image):

    ok, buf = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, 85]
    )

    if not ok:

        return Response(status=500)


    return Response(
        buf.tobytes(),
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store"}
    )


def create_app(node, state):

    app = Flask(__name__)


    @app.get("/")
    def index():
        return PAGE


    @app.get("/frame")
    def frame():

        image = node.get_frame()

        if image is None:
            return Response(status=503)

        _, lo, hi = state.snapshot()

        return jpeg(build_views(image, lo, hi)[0])


    @app.get("/maskframe")
    def maskframe():

        image = node.get_frame()

        if image is None:
            return Response(status=503)

        _, lo, hi = state.snapshot()

        return jpeg(build_views(image, lo, hi)[1])


    @app.get("/state")
    def get_state():

        target, lo, hi = state.snapshot()

        return jsonify({"target": target, "lo": lo, "hi": hi})


    @app.post("/set")
    def set_state():

        body = request.get_json(force=True)

        with state.lock:

            state.lo = [int(v) for v in body["lo"]]

            state.hi = [int(v) for v in body["hi"]]

        return jsonify({"ok": True})


    @app.get("/select")
    def select():

        target = request.args.get("target", "yellow")

        if target in PRESETS:

            state.select(target)

        return jsonify({"ok": True})


    @app.get("/sample")
    def sample():

        image = node.get_frame()

        if image is None:
            return jsonify({"ok": False})

        hls = sample_hls(
            image,
            int(request.args.get("x", 0)),
            int(request.args.get("y", 0))
        )

        state.add_sample(hls)

        return jsonify({"ok": True, "hls": hls})


    @app.get("/clear")
    def clear():

        with state.lock:

            state.samples = []

        return jsonify({"ok": True})


    @app.get("/suggest")
    def suggest():

        stats = state.sample_stats()

        if stats:

            with state.lock:

                state.lo = stats["suggest_lo"]

                state.hi = stats["suggest_hi"]

        return jsonify({"ok": stats is not None})


    @app.get("/stats")
    def stats():

        image = node.get_frame()

        _, lo, hi = state.snapshot()

        pixels = 0 if image is None else build_views(image, lo, hi)[2]

        return jsonify({
            "pixels": pixels,
            "samples": state.sample_stats(),
        })


    @app.get("/apply")
    def apply_to_node():

        target, lo, hi = state.snapshot()

        ok, message = node.push_params(target, lo, hi)

        return jsonify({"ok": ok, "message": message})


    return app


# ================================================================
# GUI 모드 (OpenCV trackbar)
# ================================================================

def run_gui(node, state):
    """noVNC(:6080) 로 접속해 보는 트랙바 창.

    키:  y/w 대상 전환   a 노드에 적용   r 기본값   q 종료
    """

    window = "HLS tuner"

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    cv2.resizeWindow(window, 980, 420)


    def on_change(_):

        with state.lock:

            for i in range(3):

                state.lo[i] = cv2.getTrackbarPos(f"lo {CHANNELS[i]}", window)

                state.hi[i] = cv2.getTrackbarPos(f"hi {CHANNELS[i]}", window)


    for i in range(3):

        cv2.createTrackbar(
            f"lo {CHANNELS[i]}", window,
            state.lo[i], CHANNEL_MAX[i], on_change
        )

        cv2.createTrackbar(
            f"hi {CHANNELS[i]}", window,
            state.hi[i], CHANNEL_MAX[i], on_change
        )


    def sync_trackbars():

        for i in range(3):

            cv2.setTrackbarPos(f"lo {CHANNELS[i]}", window, state.lo[i])

            cv2.setTrackbarPos(f"hi {CHANNELS[i]}", window, state.hi[i])


    banner = ""


    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.02)

        image = node.get_frame()


        if image is None:

            canvas = np.zeros((240, 640, 3), np.uint8)

            cv2.putText(
                canvas, "waiting for frames...", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
            )

        else:

            target, lo, hi = state.snapshot()

            blended, mask, pixels = build_views(image, lo, hi)

            canvas = np.hstack([
                blended,
                cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            ])

            cv2.putText(
                canvas,
                f"{target}  lo{lo} hi{hi}  {pixels}px",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1
            )

            cv2.putText(
                canvas,
                banner or "y/w: target   a: apply to node   r: reset   q: quit",
                (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (150, 200, 150), 1
            )


        cv2.imshow(window, canvas)

        key = cv2.waitKey(1) & 0xFF


        if key == ord("q"):

            break

        elif key in (ord("y"), ord("w")):

            state.select("yellow" if key == ord("y") else "white")

            sync_trackbars()

            banner = ""

        elif key == ord("r"):

            state.select(state.snapshot()[0])

            sync_trackbars()

            banner = ""

        elif key == ord("a"):

            target, lo, hi = state.snapshot()

            ok, message = node.push_params(target, lo, hi)

            banner = ("OK: " if ok else "FAIL: ") + message


    cv2.destroyAllWindows()


# ================================================================
# Web 모드
# ================================================================

def run_web(node, state, port):

    logging.getLogger("werkzeug").setLevel(logging.ERROR)


    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


    try:

        probe.bind(("", port))

    except OSError:

        node.get_logger().error(
            f"port {port} is already in use."
        )

        return

    finally:

        probe.close()


    server = make_server(
        "0.0.0.0", port,
        create_app(node, state),
        threaded=True
    )

    threading.Thread(
        target=server.serve_forever,
        daemon=True
    ).start()


    node.get_logger().info(
        f"HLS tuner: http://localhost:{port}"
    )


    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        server.shutdown()


# ================================================================
# Main
# ================================================================

def main():

    parser = argparse.ArgumentParser(
        description="HLS 범위 측정/튜닝 도구"
    )

    parser.add_argument(
        "--gui", action="store_true",
        help="OpenCV 트랙바 창 (기본은 웹)"
    )

    parser.add_argument(
        "--topic", default=DEFAULT_TOPIC,
        help=f"입력 영상 토픽 (기본 {DEFAULT_TOPIC})"
    )

    parser.add_argument(
        "--node", default=DEFAULT_NODE,
        help=f"적용 대상 노드 (기본 {DEFAULT_NODE})"
    )

    parser.add_argument(
        "--target", default="yellow", choices=sorted(PRESETS),
        help="시작 대상 색 (기본 yellow)"
    )

    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"웹 모드 포트 (기본 {DEFAULT_PORT})"
    )

    args, _ = parser.parse_known_args()


    rclpy.init()

    node = HlsTunerNode(args.topic, args.node)

    state = TunerState(args.target)


    try:

        if args.gui:

            run_gui(node, state)

        else:

            run_web(node, state, args.port)

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
