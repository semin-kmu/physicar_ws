# 테스트 배치

| 디렉터리 | 레벨 | 실행처 | ROS |
| --- | --- | --- | --- |
| `unit/` | L0·L1 | 호스트 / 컨테이너 | 불필요 (`core/` 만 대상) |
| `stub/` | L2 보조 | 동일 | 타 노드 스텁 (`docs/10` section 5) |
| `integration/` | L2+ | **차량 PC** | `launch_testing` |

시나리오 번호는 `docs/10` 색인과 일치시킬 것.
