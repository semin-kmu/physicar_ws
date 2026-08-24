# data

## archive/ — 예전 기록 궤적 (2026-08-24 에 치웠다)

`inner_1..3.csv` · `outer_1..3.csv` 는 `archive/` 로 옮겼다. 경로를 CAD 에서
뽑게 된 뒤로 (README 4.1.3) 이 궤적은 씨앗으로 쓰지 않고, 편집기 화면만
어지럽혔기 때문이다. **지우지는 않았다** — 나중에 실차 궤적과 비교할 때 쓴다.

편집기는 `data/<lane>_<n>.csv` 만 훑으므로 `archive/` 는 안 읽는다. 다시
보고 싶으면 파일을 `data/` 로 옮기고 편집기에서 `기록 궤적 표시`(단축키 `T`)를
켜면 된다. 새로 기록하면 번호는 다시 1 번부터 채워진다 — 옛 파일과 이름이
겹치므로 `archive/` 안에서만 겹치지 않게 관리한다.

## variants/ — 변형 경로 그림

`gen_lane_variants.py --plot` 이 그리는 PNG 다 (README 4.7). 다시 그릴 수
있는 산출물이라 없어도 된다.

## 여기 들어오는 것

`record_trajectory.py` 가 남기는 주행 궤적 CSV 가 여기 들어간다.
**바퀴 하나가 파일 하나다.** 편집기(`../web/lane_editor.html`)가
`../data/<lane>_<n>.csv` 를 훑어서 전부 읽고 평균낸다.

| 파일 | 뜻 |
|---|---|
| `inner_1.csv` `inner_2.csv` ... | 안쪽 차로 완주 바퀴. **평균 대상** |
| `outer_1.csv` `outer_2.csv` ... | 바깥쪽 차로 완주 바퀴. **평균 대상** |
| `<lane>_partial_<k>.csv` | 한 바퀴를 못 채우고 끝난 구간. 평균에서 빠진다 |
| `<lane>_recording.csv` | 진행 중인 바퀴. 정상 종료하면 사라진다 |

열은 `t, x, y, yaw`. 단위는 **미터 / 라디안**, 프레임은 `map`.
`KauPath` 의 cm 규약은 발행 단계에서 적용한다 (README 6.8).

## 한 번 실행 = 한 바퀴

한 바퀴 돌고 `Ctrl+C`. 실행할 때마다 **비어 있는 가장 작은 번호**로 들어간다.

```bash
ros2 run kau_global_path record_trajectory.py --lane inner   # -> inner_1.csv
ros2 run kau_global_path record_trajectory.py --lane inner   # -> inner_2.csv
ros2 run kau_global_path record_trajectory.py --lane inner   # -> inner_3.csv

rm inner_2.csv                                               # 2 번이 마음에 안 들면
ros2 run kau_global_path record_trajectory.py --lane inner   # -> inner_2.csv 다시
```

멈추지 않고 계속 돌아도 된다. 시작점을 지날 때마다 그 바퀴가 닫히고 다음
번호가 이어진다.

## 잘못 돈 바퀴

**그 파일만 지우면 된다.** 편집기는 남은 것만 읽는다. 번호에 구멍이 나도
편집기가 알아서 읽고, 다음 기록이 그 구멍을 채운다.

`_partial_` 은 바퀴 번호 대역을 안 쓴다. 완주했다고 판단되면
`<lane>_<n>.csv` 로 이름만 바꾸면 그대로 평균에 들어간다.

`<lane>_recording.csv` 가 남아 있으면 지난 실행이 비정상 종료한 것이다.
다음 실행이 덮어쓰기 전에 `_partial_` 로 옮겨 두므로 주행이 통째로 사라지지 않는다.

## lane 은 섞지 말 것

안쪽과 바깥쪽을 한 세션에 넣으면 어느 구간이 어느 차로인지 라벨이 없어진다
(README 3.2). 안쪽을 다 돌고 나서 `--lane outer` 로 바꾼다.

## 재매핑하면 전부 무효다

`map` 프레임 원점은 카토그래퍼가 시작한 위치라, 재매핑하면 이 좌표가 전부
무효가 된다 (README 6.3). 현재 기준 지도는 `kau_v3` 다.
