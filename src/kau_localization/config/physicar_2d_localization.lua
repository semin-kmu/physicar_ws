-- Copyright 2026 KAU AMET Team
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
--
-- PhysiCar 2D pure localization 설정.
--
-- physicar_2d.lua 를 그대로 상속하므로 프레임/토픽/센서 튜닝은 동일하고,
-- 지도를 새로 키우는 대신 -load_state_filename 으로 읽은 frozen submap 에
-- 붙어서 위치만 추정한다. 두 파일은 같은 디렉터리에 있어야 include 가 풀린다.

include "physicar_2d.lua"

-- 최근 submap 3개만 유지. 이게 없으면 pure localization 이라도
-- 주행 시간에 비례해 submap 이 계속 쌓여 메모리와 CPU 를 먹는다.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- mapping 보다 자주 최적화해야 재측위 후 수렴이 빠르다.
POSE_GRAPH.optimize_every_n_nodes = 20

-- 전역 재측위 샘플링. 초기 pose 를 안 주고 띄웠을 때 스스로 찾아오는 속도를
-- 좌우한다. 올리면 빨리 찾지만 CPU 를 더 쓴다.
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

return options
