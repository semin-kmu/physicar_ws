#!/usr/bin/env python3
"""체커보드 카메라 캘리브레이션 (Zhang's method).

체커보드: A4, 25 mm 정사각, 11x8 칸 = **10x7 내부 코너**.
cv2.findChessboardCorners 에 넣는 pattern_size 는 칸 수가 아니라
내부 코너 수이므로 (10, 7) 이다.

두 가지 모드가 있다.

  capture    /camera/image_raw 를 구독해 체커보드가 보이는 프레임만
             골라 저장한다. 코너가 잡힐 때만, 그리고 직전 저장본과
             충분히 다른 자세일 때만 받는다.

  calibrate  저장된 이미지 폴더로 calibrateCamera 를 돌리고
             ROS camera_info yaml 을 쓴다.

사용:
  ros2 run kau_lane_detection calibrate_camera.py capture  --out shots/
  ros2 run kau_lane_detection calibrate_camera.py calibrate shots/ --yaml camera_info_real.yaml
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


# 체커보드 규격 — 바꾸려면 여기만 고친다.
PATTERN = (10, 7)          # 내부 코너 (칸 11x8)
SQUARE_M = 0.025           # 정사각 한 변 [m]

FIND_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_FAST_CHECK
)

SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def object_points():
    """체커보드 한 장의 3D 점 (z = 0 평면)."""
    objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2)
    return objp * SQUARE_M


def subpix_window(corners):
    """cornerSubPix 창 반폭을 칸 간격에서 정한다.

    OpenCV 의 winSize 는 **반폭**이라 (11,11) 이면 실제 창이 23x23 이다.
    480x360 에서 25 mm 칸은 20 px 안팎이라, 고정 11 을 쓰면 창이 칸보다
    커져 이웃 코너가 섞이고 refine 이 오히려 어긋난다 (합성 검증에서
    fx 262 -> 211 로 20% 틀어졌다).

    한 행의 이웃 코너 간격 중앙값을 재서 그 절반 아래로 잡는다.
    """
    pts = corners.reshape(PATTERN[1], PATTERN[0], 2)
    dx = np.linalg.norm(np.diff(pts, axis=1), axis=2)
    dy = np.linalg.norm(np.diff(pts, axis=0), axis=2)
    spacing = float(np.median(np.concatenate([dx.ravel(), dy.ravel()])))
    half = int(max(2, min(11, spacing / 2.0 - 1.0)))
    return (half, half)


def find_corners(gray):
    """코너를 찾고 서브픽셀로 다듬는다. 실패하면 None."""
    ok, corners = cv2.findChessboardCorners(gray, PATTERN, FIND_FLAGS)
    if not ok:
        return None
    win = subpix_window(corners)
    return cv2.cornerSubPix(gray, corners, win, (-1, -1), SUBPIX_CRIT)


def pose_signature(corners, w, h):
    """자세를 대충 요약한 벡터. 중복 촬영을 거르는 데만 쓴다."""
    pts = corners.reshape(-1, 2)
    cx, cy = pts.mean(axis=0)
    span_x = pts[:, 0].max() - pts[:, 0].min()
    span_y = pts[:, 1].max() - pts[:, 1].min()
    # 기울기: 첫 행의 방향
    v = pts[PATTERN[0] - 1] - pts[0]
    ang = np.degrees(np.arctan2(v[1], v[0]))
    return np.array([cx / w, cy / h, span_x / w, span_y / h, ang / 180.0])


# ----------------------------------------------------------------------
# capture
# ----------------------------------------------------------------------

def cmd_capture(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    os.makedirs(args.out, exist_ok=True)

    state = {'saved': 0, 'sigs': [], 'last_msg': None}

    rclpy.init()
    node = Node('kau_calib_capture')

    def to_bgr(msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = arr.reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            return img[:, :, ::-1].copy()
        return img.copy()

    def cb(msg):
        state['last_msg'] = msg

    node.create_subscription(Image, args.topic, cb, 10)

    node.get_logger().info(
        f"{args.topic} 구독. 체커보드 {PATTERN[0]}x{PATTERN[1]} 내부 코너, "
        f"{SQUARE_M * 1000:.0f} mm. 목표 {args.count}장. "
        f"자세를 계속 바꿔 가며 들고 있을 것."
    )

    import time
    t_last = 0.0
    try:
        while rclpy.ok() and state['saved'] < args.count:
            rclpy.spin_once(node, timeout_sec=0.1)
            msg = state['last_msg']
            if msg is None:
                continue
            now = time.time()
            if now - t_last < args.interval:
                continue

            bgr = to_bgr(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners = find_corners(gray)
            if corners is None:
                continue

            h, w = gray.shape
            sig = pose_signature(corners, w, h)
            if any(np.linalg.norm(sig - s) < args.min_diff for s in state['sigs']):
                continue

            path = os.path.join(args.out, f"calib_{state['saved']:03d}.png")
            cv2.imwrite(path, bgr)
            state['sigs'].append(sig)
            state['saved'] += 1
            t_last = now
            node.get_logger().info(
                f"[{state['saved']}/{args.count}] 저장 {path}  "
                f"중심 ({sig[0]:.2f},{sig[1]:.2f}) 크기 {sig[2]:.2f}x{sig[3]:.2f} "
                f"기울기 {sig[4] * 180:+.0f}deg"
            )
    except KeyboardInterrupt:
        pass

    node.get_logger().info(f"{state['saved']}장 저장 완료 -> {args.out}")
    rclpy.shutdown()
    return 0


# ----------------------------------------------------------------------
# calibrate
# ----------------------------------------------------------------------

def cmd_calibrate(args):
    files = sorted(
        f for ext in ('png', 'jpg', 'jpeg', 'bmp')
        for f in glob.glob(os.path.join(args.folder, f'*.{ext}'))
    )
    if not files:
        print(f"이미지가 없다: {args.folder}", file=sys.stderr)
        return 1

    objp = object_points()
    obj_pts, img_pts, used, size = [], [], [], None

    print(f"체커보드 {PATTERN[0]}x{PATTERN[1]} 내부 코너, {SQUARE_M * 1000:.0f} mm")
    print(f"이미지 {len(files)}장 검사\n")

    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"  {os.path.basename(f):24s} 읽기 실패")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gray.shape[::-1]
        elif gray.shape[::-1] != size:
            print(f"  {os.path.basename(f):24s} 해상도 불일치 {gray.shape[::-1]} != {size} — 건너뜀")
            continue
        corners = find_corners(gray)
        if corners is None:
            print(f"  {os.path.basename(f):24s} 코너 못 찾음")
            continue
        obj_pts.append(objp)
        img_pts.append(corners)
        used.append(f)
        print(f"  {os.path.basename(f):24s} OK")

    n = len(used)
    print(f"\n사용 {n}/{len(files)}장, 해상도 {size[0]}x{size[1]}")
    if n < args.min_images:
        print(f"\n장수가 부족하다 (최소 {args.min_images}). 더 찍을 것.", file=sys.stderr)
        return 1

    # 코너가 화면을 얼마나 덮었는지 — 가장자리를 안 덮으면 왜곡계수가 안 잡힌다.
    allpts = np.concatenate([p.reshape(-1, 2) for p in img_pts])
    gx = np.clip((allpts[:, 0] / size[0] * 4).astype(int), 0, 3)
    gy = np.clip((allpts[:, 1] / size[1] * 4).astype(int), 0, 3)
    grid = np.zeros((4, 4), int)
    for x, y in zip(gx, gy):
        grid[y, x] += 1
    print("\n화면 4x4 구역별 코너 수 (0 인 칸이 있으면 그쪽을 더 찍을 것):")
    for row in grid:
        print("   " + " ".join(f"{v:5d}" for v in row))
    empty = int((grid == 0).sum())
    if empty:
        print(f"  ** 빈 구역 {empty}개 — 특히 모서리가 비면 k1/k2 가 안 잡힌다 **")

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, size, None, None
    )

    # 이미지별 재투영 RMS — 튀는 장을 골라낸다.
    #
    # cv2.norm(..., NORM_L2) 는 sqrt(sum of squared diffs) 라
    # 점 수로 나누면 RMS 가 아니다. sqrt(N) 으로 나눠야 위의
    # 전체 RMS 와 같은 단위가 된다.
    errs = []
    for i in range(n):
        proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, D)
        e = cv2.norm(img_pts[i], proj, cv2.NORM_L2) / np.sqrt(len(proj))
        errs.append(e)
    errs = np.array(errs)

    print(f"\n=== 결과 ===")
    print(f"  RMS 재투영 오차 : {rms:.4f} px")
    print(f"  이미지별 오차   : 중앙 {np.median(errs):.4f}  최대 {errs.max():.4f} px")
    worst = np.argsort(errs)[::-1][:3]
    for i in worst:
        print(f"      {os.path.basename(used[i]):24s} {errs[i]:.4f}")
    if rms > args.rms_warn:
        print(f"  ** RMS 가 {args.rms_warn} px 를 넘는다. 흐린 장이나 "
              f"체커보드 규격 불일치를 의심할 것 **")

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    print(f"\n  fx {fx:.4f}  fy {fy:.4f}   (비 {fx / fy:.4f})")
    print(f"  cx {cx:.4f}  cy {cy:.4f}   (영상 중심 {size[0]/2:.1f}, {size[1]/2:.1f})")
    print(f"  D  {np.array2string(D.ravel(), precision=6, separator=', ')}")
    fovx = 2 * np.degrees(np.arctan(size[0] / (2 * fx)))
    fovy = 2 * np.degrees(np.arctan(size[1] / (2 * fy)))
    print(f"  수평 FOV {fovx:.1f}도 / 수직 FOV {fovy:.1f}도")

    if args.yaml:
        write_camera_info(args.yaml, size, K, D, args.camera_name, rms, n)
        print(f"\n  -> {args.yaml} 기록")
    return 0


def write_camera_info(path, size, K, D, camera_name, rms, n_images):
    """ROS camera_info_manager 표준 yaml.

    D 를 그대로 싣는다 — 이 값을 받은 인지 노드가 remap 으로 편다.
    P 는 K 와 같게 둔다 (rectify 를 따로 안 하므로).
    """
    k = K.ravel()
    d = D.ravel()
    with open(path, 'w') as f:
        f.write(
            "# ============================================================\n"
            "# 체커보드 캘리브레이션 결과 (Zhang's method)\n"
            "#\n"
            f"#   체커보드 : {PATTERN[0]}x{PATTERN[1]} 내부 코너, "
            f"{SQUARE_M * 1000:.0f} mm 정사각 (A4)\n"
            f"#   이미지   : {n_images}장\n"
            f"#   RMS      : {rms:.4f} px\n"
            "#\n"
            "# scripts/calibrate_camera.py 가 생성했다. 손으로 고치지 말 것 —\n"
            "# 값을 바꾸려면 다시 촬영해서 다시 돌린다.\n"
            "#\n"
            "# D 가 0 이 아니면 이 영상에 아직 왜곡이 남아 있다는 뜻이고,\n"
            "# 인지 노드의 remap 이 그만큼을 편다. 드라이버가 이미 왜곡을\n"
            "# 편 영상을 찍었다면 여기 D 는 그 **잔여분**이다.\n"
            "# ============================================================\n\n"
        )
        f.write(f"image_width: {size[0]}\n")
        f.write(f"image_height: {size[1]}\n")
        f.write(f"camera_name: {camera_name}\n\n")
        f.write("camera_matrix:\n  rows: 3\n  cols: 3\n  data: [")
        f.write(", ".join(f"{v:.6f}" for v in k))
        f.write("]\n\n")
        f.write("distortion_model: plumb_bob\n\n")
        f.write(f"distortion_coefficients:\n  rows: 1\n  cols: {len(d)}\n  data: [")
        f.write(", ".join(f"{v:.6f}" for v in d))
        f.write("]\n\n")
        f.write(
            "rectification_matrix:\n  rows: 3\n  cols: 3\n"
            "  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n\n"
        )
        f.write("projection_matrix:\n  rows: 3\n  cols: 4\n  data: [")
        p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
        f.write(", ".join(f"{v:.6f}" for v in p))
        f.write("]\n")


# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='체커보드가 보이는 프레임을 골라 저장')
    c.add_argument('--topic', default='/camera/image_raw')
    c.add_argument('--out', default='calib_shots')
    c.add_argument('--count', type=int, default=30, help='목표 장수')
    c.add_argument('--interval', type=float, default=1.0, help='최소 저장 간격 [s]')
    c.add_argument('--min-diff', type=float, default=0.08,
                   help='직전 자세와 이만큼은 달라야 저장한다')
    c.set_defaults(func=cmd_capture)

    b = sub.add_parser('calibrate', help='이미지 폴더로 calibrateCamera')
    b.add_argument('folder')
    b.add_argument('--yaml', default=None, help='결과 camera_info yaml 경로')
    b.add_argument('--camera-name', default='camera')
    b.add_argument('--min-images', type=int, default=10)
    b.add_argument('--rms-warn', type=float, default=1.0)
    b.set_defaults(func=cmd_calibrate)

    args = ap.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
