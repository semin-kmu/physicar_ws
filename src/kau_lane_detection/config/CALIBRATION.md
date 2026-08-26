# 실차 카메라 캘리브레이션

2026-08-26 부터 **실차 intrinsic 은 체커보드 실측으로만 정한다.**
유도값·역산값은 전부 폐기했다.

## 폐기한 것과 이유

| 파일 | 무엇이었나 | 폐기 이유 |
| ---- | ---------- | --------- |
| `config/camera_info_real.yaml` | K = (261.559, 231.820, 169.603), **D = 0** | 드라이버가 왜곡을 완전히 폈다고 **가정**한 값 |
| `platform_overrides/ov5647_480x360.yaml` | 같은 K, **D ≠ 0** (잔여 배럴) | 드라이버 파이프라인을 **재현 계산**해 뽑은 값 |

두 파일이 `D` 에서 정면충돌했다. 드라이버는 `camera_info_url` 로 후자를
발행하고, 우리 launch 는 `set_camera_info` 로 전자를 덮어썼다. 어느 쪽이
이기느냐에 따라 인지 노드가 remap 을 한 번 더 하거나 안 했고, 그만큼
BEV 기하가 통째로 달라졌다. **이것이 "실차에서 틀어짐" 의 구조적 원인이다.**

## 무엇을 캘리브레이션하는가

**발행본 `/camera/image_raw` (480×360) 를 그대로 찍는다.**

드라이버는 640×480 을 캡처해 `cv::remap` 으로 한 번 편 뒤 480×360 을
낸다 (`undistort.k1 -0.3675 … dist_scale 0.7`). `dist_scale` 이 1 이 아니라
**왜곡이 덜 펴진 채로** 나온다.

그 내부 사정을 알 필요 없이, 인지가 실제로 보는 영상을 직접 재면
Zhang 이 `K` 와 **잔여 왜곡** `D` 를 한 번에 잡아 준다. 그 `D` 를 인지
노드의 remap 이 편다. 모델이 하나만 남는다.

## 체커보드

A4, **25 mm** 정사각, **11×8 칸 = 10×7 내부 코너**.

`cv2.findChessboardCorners` 의 `pattern_size` 는 칸 수가 아니라 내부
코너 수라 `(10, 7)` 이다. 규격을 바꾸면
`scripts/calibrate_camera.py` 상단의 `PATTERN` / `SQUARE_M` 만 고친다.

> 인쇄 후 **자로 칸을 재서 25 mm 가 맞는지 확인할 것.** 프린터
> 배율이 어긋나면 `fx/fy` 가 그 비율만큼 통째로 틀어지고, 그 오차가
> `sx` → `lane_width_px` → `valid_length` → `cte` 로 전부 전파된다.
> 보드는 평평한 판에 붙인다 (휘면 왜곡계수로 흡수된다).

## 절차

```bash
# 1) 촬영 — 체커보드를 들고 자세를 계속 바꾼다
ros2 run kau_lane_detection calibrate_camera.py capture \
     --out ~/calib_shots --count 30

# 2) 계산 + camera_info 생성
ros2 run kau_lane_detection calibrate_camera.py calibrate ~/calib_shots \
     --yaml src/kau_lane_detection/config/camera_info_real.yaml

# 3) 빌드 후 실차에서
ros2 launch kau_lane_detection lane_detection.launch.py platform:=real
```

`capture` 는 코너가 실제로 잡히는 프레임만, 그리고 직전 저장본과
자세가 충분히 다를 때만 받는다.

### 잘 찍는 법

`calibrate` 가 화면을 4×4 로 나눠 구역별 코너 수를 찍는다. **빈 칸이
있으면 그쪽을 더 찍어야 한다** — 특히 네 모서리가 비면 `k1/k2` 가 안
잡힌다 (합성 검증에서 모서리를 비웠더니 `fx` 가 262 → 229 로 12%
틀어졌다).

- 화면 중앙뿐 아니라 **네 모서리와 가장자리**에 보드를 두고 찍는다
- 정면뿐 아니라 **좌우·상하로 기울여** 찍는다 (기울기가 없으면
  `fx` 와 거리(tz)가 서로 상쇄돼 분리되지 않는다)
- 보드가 화면의 1/3 ~ 2/3 를 채우는 거리
- 초점이 맞고 흔들리지 않은 프레임만

### 결과 읽는 법

| 항목 | 기준 |
| ---- | ---- |
| RMS | **1 px 미만**. 넘으면 흐린 장이나 규격 불일치 |
| `fx/fy` 비 | 1.00 근처. 크게 벗어나면 픽셀이 정사각이 아니라는 뜻이라 의심 |
| `cx/cy` | 영상 중심(240, 180) 에서 크게 벗어나면 재확인 |
| 이미지별 오차 | 튀는 장은 지우고 다시 돌린다 |

## 캘리브레이션 뒤에 반드시 같이 할 것

**BEV 세 행은 intrinsic 에 종속이다.** `fy` / `cy` 가 바뀌면
`bev_vanishing_y` / `bev_src_top_y` / `bev_src_bottom_y` 를 다시 유도해야
한다 (CLAUDE.md §0-1):

```
vanishing_y = cy - fy*tan(alpha)
y(d)        = cy + fy*tan(atan(h/d) - alpha)      alpha = camera_tilt_deg + 0.284deg
```

`camera_height_cm` (14.6) 과 목표 지면거리(현재 32.35 ~ 120 cm) 는 그대로
두고 새 `fy`/`cy` 만 넣으면 된다.

## 드라이버 쪽 정리 — 완료

`/opt/physicar/src/physicar-ros/physicar_bringup/config/driver_params.yaml` 의
`camera_info_url` 을 지웠다 (2026-08-26). 그 줄은 로컬 추가분 9줄이
전부였으므로 `git checkout` 으로 원본 상태가 됐다 — 지금 그 리포에
로컬 수정은 없다.

이제 드라이버는 캘리브 파일을 안 읽고 **전부 0인 CameraInfo** 를 낸다.
유효한 값은 우리 launch 의 `set_camera_info` 가 넣는 것 하나뿐이다.
**출처가 하나가 됐다.**

### 그래서 지금 실차는 이 상태다

`config/camera_info_real.yaml` 이 아직 없으므로:

1. `set_camera_info` 가 "캘리브레이션 파일을 읽지 못했습니다" 로 종료
2. 드라이버는 0 짜리 CameraInfo 를 계속 발행
3. 인지 노드는 그걸 무효로 보고 `Waiting for CameraInfo...` 에서 멈춤

**의도한 상태다.** 캘리브레이션을 하기 전에는 실차가 돌지 않는다 —
예전처럼 출처를 알 수 없는 값으로 조용히 도는 것보다 낫다.
위 절차를 마치면 바로 풀린다.
