#!/usr/bin/env python3
"""
차선 인지 실패 프레임 수집 / 유형 분류.

두 가지 모드로 쓴다.

  수집:  ros2 run kau_lane_detection lane_failure_logger.py
  분석:  ros2 run kau_lane_detection lane_failure_logger.py --report <dir>

수집 모드는 /kau_lane_detection/status 를 보고 실패로 판정된 프레임의
원본 / BEV / debug 이미지를 저장한다. 원본을 같이 저장하는 게 핵심이다.
BEV 만 보면 "차선이 원래 없었는지" 와 "BEV 가 그 자리를 안 봤는지" 를
구분할 수 없다.

분석 모드는 저장된 원본에서 차선이 실제로 어디 있었는지를 재서
실패 유형을 나눈다. 복구 전략은 유형마다 다르다.

  A  표시없음    사다리꼴 전방 구간에 차선 표시 자체가 없음
                 -> 복구 불가. "차선 없음" 을 정직하게 보고해야 함
  B  BEV밖       차선이 영상 안에는 있는데 사다리꼴 밖
                 -> 전방 시야 축소 / 사다리꼴 조정으로 복구 가능
  C  FOV밖       차선이 영상 경계에 잘림
                 -> 카메라 pan 이 유일한 답
  D  놓침        사다리꼴 안에 차선이 보이는데도 추적 실패
                 -> HLS / 윈도우 파라미터 문제
"""

import argparse
import csv
import json
import os
import re
import sys
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


NODE_NAME = "/kau_lane_detection_node"

# 노드에서 받아올 파라미터. 분석 때 사다리꼴을 그대로 재현해야 한다.
WANT_PARAMS = [
    "bev_src_center_x", "bev_src_top_y", "bev_src_bottom_y",
    "bev_vanishing_y", "bev_src_bottom_width", "bev_dst_margin_ratio",
    "bev_out_width", "bev_out_height", "lane_width_cm", "camera_height_cm",
    "yellow_hls_lo", "yellow_hls_hi", "white_hls_lo", "white_hls_hi",
]

FALLBACK = {
    "bev_src_center_x": 240.0, "bev_src_top_y": 205.0,
    "bev_src_bottom_y": 270.0, "bev_vanishing_y": 179.0,
    "bev_src_bottom_width": 480.0, "bev_dst_margin_ratio": 0.25,
    "bev_out_width": 480, "bev_out_height": 360,
    "lane_width_cm": 35.0, "camera_height_cm": 14.65,
    "yellow_hls_lo": [15, 80, 150], "yellow_hls_hi": [35, 255, 255],
    "white_hls_lo": [0, 200, 0], "white_hls_hi": [180, 255, 70],
}


def parse_status(text):
    """'lw=8 yw=9 ...' -> dict"""
    out = {}
    for k, v in re.findall(r"(\w+)=(-?[\d.]+)", text):
        out[k] = float(v) if "." in v else int(v)
    return out


# ====================================================================
# 수집
# ====================================================================

class FailureLogger(Node):

    def __init__(self, args):
        super().__init__("lane_failure_logger")

        self.args = args
        self.bridge = CvBridge()
        self.frames = {}
        self.n_total = 0
        self.n_saved = 0
        self.tally = {}
        self.last_save = 0.0

        os.makedirs(args.out, exist_ok=True)

        self.csv_path = os.path.join(args.out, "events.csv")
        new = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline="")
        self.csv = csv.writer(self.csv_file)
        if new:
            self.csv.writerow(
                ["stamp", "reason", "lw", "yw", "rw", "lv", "yv", "rv",
                 "path", "gate", "win", "len", "rad", "maxlat",
                 "cte", "yaw", "prefix"])

        for topic, key in [
            ("/kau_lane_detection/undistorted_image", "undist"),
            ("/kau_lane_detection/bev_image", "bev"),
            ("/kau_lane_detection/debug_image", "debug"),
        ]:
            self.create_subscription(
                Image, topic,
                lambda m, k=key: self.frames.__setitem__(
                    k, self.bridge.imgmsg_to_cv2(m, "bgr8")), 10)

        self.create_subscription(
            String, "/kau_lane_detection/status", self.on_status, 10)

        self.get_logger().info(
            f"수집 시작 -> {args.out}  "
            f"(창합계<{args.min_windows} / 노랑0 / 경로없음 / "
            f"반경<{args.min_radius:.0f}cm 이면 저장)")

    # ----------------------------------------------------------------

    def classify_reason(self, s):
        """실패 사유. 여러 개면 가장 심각한 것 하나."""
        if s.get("gate", 0):
            return "gate%d" % s["gate"]
        if not s.get("path", 0):
            return "nopath"
        rad = s.get("rad", 0.0)
        if 0.0 < rad < self.args.min_radius:
            return "sharp"
        if s.get("yw", 0) == 0:
            return "noyellow"
        if s.get("lw", 0) + s.get("yw", 0) + s.get("rw", 0) \
                < self.args.min_windows:
            return "thin"
        return None

    def on_status(self, msg):
        self.n_total += 1
        s = parse_status(msg.data)
        reason = self.classify_reason(s)

        if reason is None:
            return
        if len(self.frames) < 3:
            return

        now = time.time()
        if now - self.last_save < self.args.interval:
            return
        if self.n_saved >= self.args.max_frames:
            return
        self.last_save = now

        prefix = f"{self.n_saved:04d}_{reason}"
        for key, img in self.frames.items():
            cv2.imwrite(
                os.path.join(self.args.out, f"{prefix}_{key}.png"), img)

        self.csv.writerow([
            f"{now:.3f}", reason,
            s.get("lw"), s.get("yw"), s.get("rw"),
            s.get("lv"), s.get("yv"), s.get("rv"),
            s.get("path"), s.get("gate"), s.get("win"),
            s.get("len"), s.get("rad"), s.get("maxlat"),
            s.get("cte"), s.get("yaw"), prefix])
        self.csv_file.flush()

        self.n_saved += 1
        self.tally[reason] = self.tally.get(reason, 0) + 1

        rate = 100.0 * self.n_saved / max(1, self.n_total)
        summary = " ".join(f"{k}:{v}" for k, v in sorted(self.tally.items()))
        print(f"\r저장 {self.n_saved:4d} / 프레임 {self.n_total:5d} "
              f"({rate:4.1f}%)   {summary}          ", end="", flush=True)


def fetch_params(node):
    """실행 중인 노드에서 BEV 파라미터를 그대로 받아온다."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f"{NODE_NAME}/get_parameters")
    if not cli.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn("노드 파라미터 서비스 없음. 기본값 사용.")
        return dict(FALLBACK)

    req = GetParameters.Request()
    req.names = WANT_PARAMS
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    res = fut.result()
    if res is None:
        node.get_logger().warn("파라미터 응답 없음. 기본값 사용.")
        return dict(FALLBACK)

    out = dict(FALLBACK)
    for name, v in zip(WANT_PARAMS, res.values):
        if v.type == 2:
            out[name] = int(v.integer_value)
        elif v.type == 3:
            out[name] = float(v.double_value)
        elif v.type == 7:
            out[name] = list(v.integer_array_value)
    return out


def collect(args):
    rclpy.init()
    node = FailureLogger(args)

    params = fetch_params(node)
    with open(os.path.join(args.out, "params.json"), "w") as f:
        json.dump(params, f, indent=2)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        print(f"총 {node.n_total} 프레임 중 {node.n_saved} 장 저장.")
        print(f"분석:  ros2 run kau_lane_detection lane_failure_logger.py "
              f"--report {args.out}")
        node.csv_file.close()

        # SIGTERM 으로 죽을 때 rclpy 가 이미 내려가 있을 수 있다.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


# ====================================================================
# 분석
# ====================================================================

def trapezoid_x(p, row):
    """원본 영상 row 에서 src 사다리꼴의 x 범위."""
    half = (p["bev_src_bottom_width"] * 0.5) * \
        (row - p["bev_vanishing_y"]) / \
        (p["bev_src_bottom_y"] - p["bev_vanishing_y"])
    cx = p["bev_src_center_x"]
    return cx - half, cx + half


def mask_presence(mask, p, rows, w):
    """주어진 행들에서 이 마스크가 사다리꼴 안/밖/경계 중 어디 있었나."""
    st = dict(rows=0, present=0, inside=0, outside=0, border=0)
    h = mask.shape[0]
    for row in rows:
        if row < 2 or row >= h - 2:
            continue
        st["rows"] += 1
        tl, tr = trapezoid_x(p, row)
        xs = np.where(mask[row - 2:row + 3].any(axis=0))[0]
        if len(xs) == 0:
            continue
        st["present"] += 1
        if xs.min() <= 1 or xs.max() >= w - 2:
            st["border"] += 1
        if ((xs >= tl) & (xs <= tr)).any():
            st["inside"] += 1
        else:
            st["outside"] += 1
    return st


def classify_frame(undist, p):
    """실패 유형 판정.

    판정은 노란 중앙선 기준이다. 흰선은 연석/횡단보도/포장 경계에도
    걸려서 거의 항상 보이기 때문에, 같이 세면 전부 "보이는데 놓침"
    으로 몰린다. 노란선이 세 차선 탐색의 기준점이기도 하다.

    노란선이 점선이라 원거리에서 "없음" 이 점선 공백일 수 있다.
    그래서 사다리꼴 전 구간 기준 존재 여부도 같이 본다.
    """
    h, w = undist.shape[:2]
    hls = cv2.cvtColor(undist, cv2.COLOR_BGR2HLS)
    yellow = cv2.inRange(hls, tuple(p["yellow_hls_lo"]),
                         tuple(p["yellow_hls_hi"]))
    white = cv2.inRange(hls, tuple(p["white_hls_lo"]),
                        tuple(p["white_hls_hi"]))

    top = int(p["bev_src_top_y"])
    bot = int(p["bev_src_bottom_y"])
    far = list(range(top, int(top + (bot - top) / 3.0) + 1, 2))
    allr = list(range(top, bot + 1, 2))

    y_far = mask_presence(yellow, p, far, w)
    y_all = mask_presence(yellow, p, allr, w)
    w_far = mask_presence(white, p, far, w)

    stat = dict(y_far=y_far, y_all=y_all, w_far=w_far)

    if y_far["inside"] > 0:
        kind = "D_놓침"
    elif y_far["border"] > 0:
        kind = "C_FOV밖"
    elif y_far["outside"] > 0:
        kind = "B_BEV밖"
    elif y_all["present"] > 0:
        kind = "E_점선공백"
    else:
        kind = "A_표시없음"
    return kind, stat


def report(args):
    d = args.report
    pf = os.path.join(d, "params.json")
    p = json.load(open(pf)) if os.path.exists(pf) else dict(FALLBACK)

    rows = list(csv.DictReader(open(os.path.join(d, "events.csv"))))
    if not rows:
        print("수집된 실패 프레임이 없습니다.")
        return

    types, cross = {}, {}
    for r in rows:
        img = cv2.imread(os.path.join(d, f"{r['prefix']}_undist.png"))
        if img is None:
            continue
        kind, _ = classify_frame(img, p)
        types[kind] = types.get(kind, 0) + 1
        cross.setdefault(r["reason"], {})
        cross[r["reason"]][kind] = cross[r["reason"]].get(kind, 0) + 1
        r["kind"] = kind

    n = sum(types.values())
    print(f"\n실패 프레임 {n} 장\n")
    print("유형 분포 (원본 영상에서 차선이 실제로 어디 있었나)")
    order = ["A_표시없음", "E_점선공백", "B_BEV밖", "C_FOV밖",
             "D_놓침", "E_other"]
    hint = {
        "A_표시없음": "노란선 자체가 없음. 복구 불가 -> 차선없음 보고",
        "E_점선공백": "원거리는 점선 공백. 근거리엔 노란선 있음",
        "B_BEV밖": "영상엔 있는데 사다리꼴 밖 -> 시야 축소로 복구",
        "C_FOV밖": "영상 경계에 잘림 -> pan 이 유일한 답",
        "D_놓침": "사다리꼴 안에 보이는데 실패 -> HLS/윈도우 문제",
        "E_other": "-",
    }
    for k in order:
        if k not in types:
            continue
        c = types[k]
        bar = "#" * int(40 * c / n)
        print(f"  {k:<12} {c:4d} ({100*c/n:5.1f}%) {bar:<40} {hint[k]}")

    print("\n노드가 낸 실패 사유 x 실제 유형")
    kinds = [k for k in order if k in types]
    print("  " + " " * 10 + "".join(f"{k:>12}" for k in kinds))
    for reason, m in sorted(cross.items()):
        print(f"  {reason:<10}" + "".join(f"{m.get(k,0):>12}" for k in kinds))

    sharp = [r for r in rows if r["reason"] == "sharp"]
    if sharp:
        radii = [float(r["rad"]) for r in sharp]
        print(f"\n최소회전반경 미만 경로를 발행한 프레임: {len(sharp)} 장"
              f"  (반경 {min(radii):.0f} ~ {max(radii):.0f} cm)")
        print("  -> 이만큼이 컨트롤러에 그대로 나갔을 값입니다.")


# ====================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="lane_failures", help="저장 디렉터리")
    ap.add_argument("--report", metavar="DIR", help="수집 결과 분석")
    ap.add_argument("--min-windows", type=int, default=12,
                    help="세 차선 창 합계가 이 미만이면 실패 (기본 12/27)")
    ap.add_argument("--min-radius", type=float, default=60.0,
                    help="경로 곡률반경이 이 미만이면 실패 [cm]. "
                         "휠베이스 18cm / 최대조향 20도 -> 최소 50cm")
    ap.add_argument("--interval", type=float, default=0.4,
                    help="저장 최소 간격 [s]")
    ap.add_argument("--max-frames", type=int, default=400)
    args = ap.parse_args()

    if args.report:
        report(args)
    else:
        collect(args)


if __name__ == "__main__":
    main()
