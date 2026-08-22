// KAU lane_graph 편집기.
//
// 좌표계는 셋이다. 섞으면 지도가 뒤집힌 채로 그려진다 (README 2.3).
//   world  : map 프레임, 미터. 저장/발행되는 유일한 진짜 좌표.
//   image  : pgm 픽셀. 좌상단 원점, y 아래로 증가.
//   screen : 캔버스 픽셀. image 에 view.scale / view.tx / view.ty 를 먹인 것.
//
// 곡선 규약은 kau_control/scripts/fake_path.py 와 같아야 한다. 노드에서
// (theta, kappa) 를 구하고 quintic Hermite -> Bezier 정확 기저변환으로 제어점을
// 만든다. 인접 segment 가 경계 노드의 (theta, kappa) 를 공유하므로 G2 가 자동
// 성립한다. 화면에 보이는 곡선이 곧 발행될 곡선이다.

'use strict';

const DEGREE = 5;
const NCTRL = DEGREE + 1;

// 사이드바 제목 옆에 찍는다. 브라우저가 옛 js 를 캐시하고 있는지
// 한눈에 구분하려는 것이다. lane_editor.html 의 ?v= 와 같이 올린다.
const BUILD = '20260821n';

// 차량 제원. 곡률 한계는 여기서 유도한다 — 숫자를 손으로 박아 두면 차가 바뀌었을 때
// 아무도 못 찾는다.
//
// kinematic bicycle model 에서 조향각 d 일 때 **후륜축 중심**의 회전반경은
//
//     R = L / tan(d),    kappa = tan(d) / L
//
// (순간회전중심이 후륜축 연장선 위에 있다. 전륜 중심으로 재면 L/sin(d) 라 더 크다.
//  제어기 기준점이 후륜축 중심이므로 이쪽을 쓴다.)
//
// L = 0.18 m, d = 20 deg  ->  R_min 0.4945 m, kappa_max 2.0221 1/m
//
// margin 은 그 기하학적 한계에서 얼마나 물러설지다. 조향 슬랙·명령 포화·추종
// 오차를 감안하면 한계에 붙여 설계하면 안 된다. 0.90 이면 조향 18.0 deg 에 해당한다.
const VEH = { wheelbase: 0.18, maxSteerDeg: 20, margin: 0.90 };

function kappaOfSteer(deg) {
  return Math.tan(deg * Math.PI / 180) / VEH.wheelbase;
}

function steerOfKappa(k) {
  return Math.atan(Math.abs(k) * VEH.wheelbase) * 180 / Math.PI;
}

// 기하학적 최대 곡률 (마진 없음)
function kappaMax() { return kappaOfSteer(VEH.maxSteerDeg); }

// 설계 한계. 이 값을 넘는 세그먼트는 빨갛게 표시된다.
function curvLimit() {
  const el = document.getElementById('kappaLimit');
  const v = el ? parseFloat(el.value) : NaN;
  return isNaN(v) || v <= 0 ? kappaMax() * VEH.margin : v;
}

const MAP_DIR = '../../kau_localization/maps/';
const MAP_NAME = 'kau_v3';
const DATA_DIR = '../data/';
const TRACK_JSON = '../config/amet2026_track.json';

// ---------------------------------------------------------------- 상태

// 발행되는 global path 는 **주행면 중심선 하나**다 (2026-08-21 결정, README 4.1.3).
// 차로를 둘로 나눠 보지 않는다. lane_inner / lane_outer 는 나중에 차로 개념이
// 필요해질 때를 위해 남겨 둔 것이고 지금 경로에는 안 쓴다.
// 노드 id 대역. 레이어마다 100 번대로 띄워서 번호만 보고 어느 레이어인지 알게 한다.
const LAYER_BASE = { center: 0, lane_inner: 100, lane_outer: 200, bnd_outer: 300, bnd_inner: 400 };

const LAYER_DEFS = [
  { id: 'center', label: '주행면 중심선 (global path)', color: '#4ea3ff', kind: 'center', route: 'center_loop' },
  { id: 'lane_inner', label: '안쪽 차로 중심선 (미사용)', color: '#63d0a8', kind: 'lane', lane: 'inner', route: 'inner_loop' },
  { id: 'lane_outer', label: '바깥쪽 차로 중심선 (미사용)', color: '#ffb24e', kind: 'lane', lane: 'outer', route: 'outer_loop' },
  { id: 'bnd_outer', label: 'outer boundary (ROI)', color: '#5ac878', kind: 'boundary', side: 'outer' },
  { id: 'bnd_inner', label: 'inner boundary (ROI)', color: '#c878e0', kind: 'boundary', side: 'inner' },
];

const S = {
  map: null,               // {w,h,res,ox,oy,name,canvas}
  laps: { inner: [], outer: [] },         // [{name, pts}] — 파일 하나가 한 바퀴
  lapSel: { inner: 'avg', outer: 'avg' }, // 'avg' | 바퀴 index
  lapAvg: { inner: null, outer: null },   // alignLaps 캐시
  layers: {},              // id -> {pts:[[x,y]], closed, visible}
  active: 'center',
  view: { scale: 1, tx: 0, ty: 0 },
  show: { map: true, grid: true, traj: true, curve: false, ctrl: false },
  undo: [], redo: [],
  hover: null,             // {layer, index}
  drag: null,
  cursor: null,            // [wx, wy]
  check: null,
  segSel: -1,              // 세부를 펼쳐 볼 세그먼트

  // 참조 이미지. kau_v3 지도에는 벽밖에 안 찍힌다 — 차선은 바닥 테이프라
  // LiDAR 에 안 잡히기 때문이다 (README 4.3). 그래서 트랙 도면이나 위에서 찍은
  // 사진을 깔고 벽에 맞춘 뒤 그 위에서 노드를 찍는다.
  //   cx, cy : 이미지 중심의 map 좌표 (m)
  //   wm     : 이미지 가로 폭 (m). 세로는 원본 비율로 따라간다
  //   rot    : map 프레임 기준 반시계 회전 (rad)
  overlay: { img: null, name: '', cx: 0, cy: 0, wm: 1, rot: 0, alpha: 0.6, on: true, fit: false },

  // 시뮬레이터 world 에서 뽑아 온 AMET 2026 트랙 CAD (scripts/extract_sim_track.py).
  // 사진이 아니라 도형이라 확대해도 안 뭉개지고, 크기가 이미 미터라 맞출 게
  // 원점과 회전뿐이다.
  track: { data: null, ox: 3.68, oy: 1.39, rot: 0, alpha: 0.85, on: true, road: true },
};

for (const d of LAYER_DEFS) {
  S.layers[d.id] = { pts: [], closed: true, visible: true };
}

// 화면에 숫자를 보여줄 때만 쓰는 좌표계. 실제 트랙을 보는 방향과 map 프레임이
// 어긋나서 읽기 불편하다는 요청으로 넣었다.
//
//   X = y - oy,   Y = x - ox        (x <-> y 맞바꿈 + 원점 이동)
//
// **저장·내보내기 좌표는 언제나 map 프레임 미터다** (README 4.2). 이 변환은
// HUD 와 격자 라벨에만 걸리고, 노드 좌표에는 절대 닿지 않는다. 여기서 좌표를
// 바꿔 저장하면 lane_graph 가 TF 와 어긋나 전부 무효가 된다 (6.3).
const DISP = { on: true, ox: 3.68, oy: -1.39 };

function toDisp(p) { return [p[1] - DISP.oy, p[0] - DISP.ox]; }

const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');

// ---------------------------------------------------------------- 기하

function sub(a, b) { return [a[0] - b[0], a[1] - b[1]]; }
function dist(a, b) { return Math.hypot(a[0] - b[0], a[1] - b[1]); }

// 노드마다 (theta, kappa) 를 구한다.
//
// 연속한 세 점을 지나는 이차식을 chord-length 매개변수 위에서 세우고 미분한다.
// 등간격 t 로 잡으면 노드 간격이 불균일한 구간에서 접선이 한쪽으로 끌린다.
// 끝점(개곡선)에서는 창을 안쪽으로 밀고 그 t 위치에서 평가한다 — 한쪽 차분보다
// 곡률이 안정적이다.
function knotFrames(pts, closed) {
  const n = pts.length;
  if (n < 2) return [];

  if (n === 2) {
    const th = Math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]);
    return [{ p: pts[0], th, k: 0 }, { p: pts[1], th, k: 0 }];
  }

  const frames = [];

  for (let i = 0; i < n; i++) {
    let a, b, c, at;   // 창 인덱스 세 개와, 평가할 위치

    if (closed) {
      a = (i - 1 + n) % n; b = i; c = (i + 1) % n; at = 1;
    } else if (i === 0) {
      a = 0; b = 1; c = 2; at = 0;
    } else if (i === n - 1) {
      a = n - 3; b = n - 2; c = n - 1; at = 2;
    } else {
      a = i - 1; b = i; c = i + 1; at = 1;
    }

    const p0 = pts[a], p1 = pts[b], p2 = pts[c];
    const t1 = dist(p1, p0);
    const t2 = t1 + dist(p2, p1);

    if (t1 < 1e-9 || t2 - t1 < 1e-9) {
      // 겹친 노드. 이웃에서 베껴 쓰고 넘어간다.
      frames.push({ p: pts[i], th: frames.length ? frames[frames.length - 1].th : 0, k: 0 });
      continue;
    }

    // Newton 분할차분: P(t) = p0 + f01 (t-t0) + f012 (t-t0)(t-t1)
    const f01 = [(p1[0] - p0[0]) / t1, (p1[1] - p0[1]) / t1];
    const f12 = [(p2[0] - p1[0]) / (t2 - t1), (p2[1] - p1[1]) / (t2 - t1)];
    const f012 = [(f12[0] - f01[0]) / t2, (f12[1] - f01[1]) / t2];

    const t = at === 0 ? 0 : (at === 1 ? t1 : t2);
    const d1 = [f01[0] + f012[0] * (2 * t - t1), f01[1] + f012[1] * (2 * t - t1)];

    const th = Math.atan2(d1[1], d1[0]);

    // 곡률은 이차식의 미분이 아니라 세 점의 외접원에서 뽑는다 (Menger 곡률).
    //   k = 4 * (부호있는 삼각형 넓이) / (|AB| |BC| |CA|)
    // 원호 위의 세 점이면 정확히 1/R 이 나온다. 이차식 미분은 chord-length
    // 매개변수가 호길이가 아니라서 같은 배치에서 4~17% 씩 곡률을 부풀린다.
    // 실측 궤적은 코너에서 거의 원호라 이 차이가 그대로 곡률 경고 오판이 된다.
    const cross = (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0]);
    const ca = dist(p2, p0);
    const denom = t1 * (t2 - t1) * ca;
    const k = denom < 1e-12 ? 0 : 2 * cross / denom;

    frames.push({ p: pts[i], th, k });
  }

  return frames;
}

// (theta, kappa) -> (P', P''). sigma 는 재매개변수화 배율.
function stateDerivs(th, k, sigma) {
  const tx = Math.cos(th), ty = Math.sin(th);
  const nx = -Math.sin(th), ny = Math.cos(th);
  return [
    [sigma * tx, sigma * ty],
    [sigma * sigma * k * nx, sigma * sigma * k * ny],
  ];
}

// quintic Hermite -> Bezier 제어점 6개. fake_path.py 의 hermite_to_bezier 와 같은 식.
//
// scale 은 재매개변수화 배율 sigma 에만 걸린다. sigma 를 바꿔도 양 끝의
// (theta, kappa) 는 그대로다 — stateDerivs 가 P'=sigma·t, P''=sigma²·k·n 이라
// kappa = |P'xP''|/|P'|³ 에서 sigma 가 약분되기 때문이다. 그래서 이 값은
// C² 를 건드리지 않고 곡선의 "부푸는 정도" 만 조절하는 자유도가 된다.
function hermiteToBezier(f0, f1, scale = 1) {
  const p0 = f0.p, p1 = f1.p;
  const chord = dist(p1, p0) * scale;
  const [A, B] = stateDerivs(f0.th, f0.k, chord);
  const [C, D] = stateDerivs(f1.th, f1.k, chord);
  return [
    [p0[0], p0[1]],
    [p0[0] + A[0] / 5, p0[1] + A[1] / 5],
    [p0[0] + 2 * A[0] / 5 + B[0] / 20, p0[1] + 2 * A[1] / 5 + B[1] / 20],
    [p1[0] - 2 * C[0] / 5 + D[0] / 20, p1[1] - 2 * C[1] / 5 + D[1] / 20],
    [p1[0] - C[0] / 5, p1[1] - C[1] / 5],
    [p1[0], p1[1]],
  ];
}

function buildSegments(pts, closed) {
  const frames = knotFrames(pts, closed);
  if (frames.length < 2) return [];

  const segs = [];
  const n = frames.length;
  const last = closed ? n : n - 1;

  for (let i = 0; i < last; i++) {
    segs.push(hermiteToBezier(frames[i], frames[(i + 1) % n]));
  }

  return segs;
}

// ---------------------------------------------------------------- 세그먼트

// 여기부터가 "내가 직접 나눈 세그먼트" 다. 위의 buildSegments 는 노드쌍마다
// Bezier 를 하나씩 뽑는 예전 방식이고, 경계를 하나도 안 찍으면 그대로 쓴다.
//
// 규약
//   경계 노드(brk) 와 경계 노드 사이가 세그먼트 하나이고, 세그먼트 하나가
//   quintic Bezier 하나 = 제어점 6 개다. ctrl[0] · ctrl[5] 가 곧 경계 노드이고
//   가운데 4 개는 아래 규칙으로 유도된다. 노드는 여전히 제어점이 아니다.
//
//   노드 2 개  line : κ=0, θ=현 방향. 제어점 6 개가 전부 현 위에 놓여서
//                     화면에서도 발행에서도 정확히 직선이다.
//   노드 2 개  link : 내가 '전이' 로 표시한 것. 양 끝 (θ,κ) 를 이웃 세그먼트에서
//                     그대로 받는다. 직선과 원호를 C² 로 잇는 유일한 방법이다.
//                     (직선 κ=0 과 원호 κ=1/R 은 맞닿는 한 절대 C² 가 안 된다.)
//   노드 3 개  arc  : 세 점의 외접원. 양 끝 κ = ±1/R 로 같다. 가운데 노드를
//                     끌면 곡률이 직접 바뀐다.
//   노드 4 개+ free : 예전 방식. 세그먼트 안에서 노드쌍마다 Bezier 를 뽑는다.

function wrapPi(a) {
  return ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
}

function circumcenter(a, b, c) {
  const d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
  if (Math.abs(d) < 1e-12) return null;
  const a2 = a[0] * a[0] + a[1] * a[1];
  const b2 = b[0] * b[0] + b[1] * b[1];
  const c2 = c[0] * c[0] + c[1] * c[1];
  return [(a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d,
          (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d];
}

function lineFrames(p0, p1) {
  const th = Math.atan2(p1[1] - p0[1], p1[0] - p0[0]);
  return [{ p: p0, th, k: 0 }, { p: p1, th, k: 0 }];
}

// p0 -> pm -> p1 순서로 지나는 원호. 양 끝의 (θ, κ) 와 회전각을 준다.
// κ 부호는 knotFrames 와 같다 — 좌회전이 +.
function arcFrames(p0, pm, p1) {
  const O = circumcenter(p0, pm, p1);
  if (!O) return null;
  const R = dist(p0, O);
  if (!(R > 1e-9) || !isFinite(R)) return null;

  const cross = (pm[0] - p0[0]) * (p1[1] - pm[1]) - (pm[1] - p0[1]) * (p1[0] - pm[0]);
  const s = cross >= 0 ? 1 : -1;

  // 접선은 반지름의 수직. 진행 방향 부호 s 를 곱한다.
  const tang = (p) => Math.atan2(s * (p[0] - O[0]), -s * (p[1] - O[1]));

  // 회전각. 90도를 넘어가면 quintic 한 개로는 원호를 잘 못 흉내낸다.
  const ang = (p) => Math.atan2(p[1] - O[1], p[0] - O[0]);
  let sweep = s * wrapPi(ang(p1) - ang(p0));
  if (sweep < 0) sweep += 2 * Math.PI;

  return {
    frames: [{ p: p0, th: tang(p0), k: s / R }, { p: p1, th: tang(p1), k: s / R }],
    R, sweep, O,
  };
}

// 원호 세그먼트의 sigma 배율을 고른다.
//
// (theta, kappa) 를 원의 값으로 고정한 quintic 은 회전각이 커질수록 원에서
// 벌어진다 (90도에서 2.5 cm, 150도면 19 cm). sigma 는 C² 를 안 건드리므로
// 이걸 자유도로 써서 진짜 원에서 가장 덜 벗어나는 값을 찾는다.
// 목적함수는 "샘플점의 반지름 오차 최대값" 이다 — 가운데 노드 하나만 맞추면
// 나머지가 출렁일 수 있다.
// 드래그 중에는 같은 원호가 프레임마다 다시 들어온다. 값이 안 바뀐 세그먼트까지
// 매번 황금분할을 돌리면 노드 20 개짜리 트랙에서 한 프레임이 100 ms 를 넘는다.
const ARC_CACHE = new Map();

function fitArcScale(f0, f1, O, R) {
  const key = [f0.p[0], f0.p[1], f1.p[0], f1.p[1], f0.th, f1.th, f0.k]
    .map((v) => v.toFixed(6)).join(',');
  const hit = ARC_CACHE.get(key);
  if (hit !== undefined) return hit;

  const err = (scale) => {
    const c = hermiteToBezier(f0, f1, scale);
    let e = 0;
    for (let i = 1; i < 16; i++) e = Math.max(e, Math.abs(dist(deCasteljau(c, i / 16), O) - R));
    return e;
  };

  // 황금분할. 구간은 넉넉히 잡되 뒤집히지 않게 양수로 제한한다.
  let lo = 0.4, hi = 2.5;
  const g = (Math.sqrt(5) - 1) / 2;
  let a = hi - g * (hi - lo), b = lo + g * (hi - lo);
  let fa = err(a), fb = err(b);

  // 25 회면 구간이 1e-5 배로 줄어든다. 그보다 잘게 찾아 봐야 의미가 없다.
  for (let i = 0; i < 25; i++) {
    if (fa < fb) { hi = b; b = a; fb = fa; a = hi - g * (hi - lo); fa = err(a); }
    else { lo = a; a = b; fa = fb; b = lo + g * (hi - lo); fb = err(b); }
  }

  const best = fa < fb ? a : b;
  if (ARC_CACHE.size > 4000) ARC_CACHE.clear();
  ARC_CACHE.set(key, best);
  return best;
}

// 노드마다 붙는 플래그. 예전 파일에는 없으니 없으면 전부 false 로 채운다.
// 전부 false = 경계를 안 찍은 상태 = 예전 동작.
function flagArray(L, key) {
  const n = L.pts.length;
  const a = Array.isArray(L[key]) ? L[key].slice(0, n).map(Boolean) : [];
  while (a.length < n) a.push(false);
  return a;
}

function brkFlags(L) {
  const b = flagArray(L, 'brk');
  if (!L.closed && b.length >= 2) { b[0] = true; b[b.length - 1] = true; }
  return b;
}

function linkFlags(L) { return flagArray(L, 'link'); }

// 경계 노드로 잘라 세그먼트 목록을 만든다. 나눌 게 없으면 null — 호출한 쪽이
// 예전 경로(buildSegments)로 떨어지라는 뜻이다.
function splitSegments(L) {
  const n = L.pts.length;
  if (n < 2) return null;

  // 판정은 내가 실제로 찍은 플래그로 한다. brkFlags 는 열린 곡선의 양 끝을
  // 자동으로 경계로 만들기 때문에, 그걸로 세면 "아무것도 안 찍음" 과
  // "양 끝만 찍음" 이 구분되지 않는다.
  const raw = flagArray(L, 'brk');
  const rawCount = raw.filter(Boolean).length;
  if (L.closed ? rawCount < 2 : rawCount < 1) return null;

  const b = brkFlags(L);
  const lk = linkFlags(L);
  const marks = [];
  for (let i = 0; i < n; i++) if (b[i]) marks.push(i);

  if (marks.length < 2) return null;

  const segs = [];
  const last = L.closed ? marks.length : marks.length - 1;

  for (let j = 0; j < last; j++) {
    const a = marks[j], c = marks[(j + 1) % marks.length];
    const idx = [a];
    let i = a;
    do { i = (i + 1) % n; idx.push(i); } while (i !== c);
    const type = idx.length === 2 ? (lk[a] ? 'link' : 'line')
      : (idx.length === 3 ? 'arc' : 'free');
    segs.push({ idx, type });
  }

  return segs;
}

// 레이어 하나를 통째로 푼다.
//
//   { segs:   [{idx, type, frames, ctrl:[[6점]...], dev, sweep, R}],
//     joints: [{node, seg, dth, dk}],
//     legacy: 경계를 안 찍어서 예전 방식으로 푼 것 }
//
// joints 가 C² 검증 결과다. dth·dk 는 경계에서 들어오는 쪽과 나가는 쪽이
// 제안한 (θ, κ) 의 차이다. 둘 다 0 이면 그 자리는 C² 다.
function buildLayer(L, opts = {}) {
  const n = L.pts.length;
  const out = { segs: [], joints: [], legacy: false, closed: !!L.closed };
  if (n < 2) return out;

  const parts = splitSegments(L);

  if (!parts) {
    out.legacy = true;
    out.segs = buildSegments(L.pts, L.closed).map((c, i) => ({
      idx: [i, (i + 1) % n], type: 'free', ctrl: [c], dev: 0,
    }));
    return out;
  }

  const m = parts.length;

  // 1) 세그먼트마다 자기 양 끝 (θ, κ) 를 제안한다. link 는 제안하지 않는다.
  const prop = parts.map((s) => {
    const P = s.idx.map((i) => L.pts[i]);
    if (s.type === 'link') return null;
    if (s.type === 'line') return lineFrames(P[0], P[1]);
    if (s.type === 'arc') {
      const a = arcFrames(P[0], P[1], P[2]);
      if (!a) return lineFrames(P[0], P[2]);          // 세 점이 일직선
      s.R = a.R; s.sweep = a.sweep; s.O = a.O;
      return a.frames;
    }
    s.inner = knotFrames(P, false);
    return [s.inner[0], s.inner[s.inner.length - 1]];
  });

  // 2) link 는 이웃의 제안을 받아 쓴다. 그래서 양쪽 다 C² 로 붙는다.
  for (let j = 0; j < m; j++) {
    if (prop[j]) continue;
    const P = parts[j].idx.map((i) => L.pts[i]);
    const ln = lineFrames(P[0], P[1]);
    const prev = (out.closed || j > 0) ? prop[(j - 1 + m) % m] : null;
    const next = (out.closed || j < m - 1) ? prop[(j + 1) % m] : null;
    prop[j] = [
      prev ? { p: P[0], th: prev[1].th, k: prev[1].k } : ln[0],
      next ? { p: P[1], th: next[0].th, k: next[0].k } : ln[1],
    ];
  }

  // 3) 경계마다 양쪽 제안을 비교한다. 이게 C² 검증이다.
  const res = prop.map((f) => [{ ...f[0] }, { ...f[1] }]);
  const blend = !!opts.blend;
  const jlast = out.closed ? m : m - 1;

  for (let j = 0; j < jlast; j++) {
    const a = res[j][1];
    const b = res[(j + 1) % m][0];
    const dth = wrapPi(b.th - a.th);
    const dk = b.k - a.k;
    const p = parts[j];
    // dth/dk 는 "양쪽이 원래 제안한 값의 차이" 다. blend 를 켜면 그 차이를
    // 평균으로 없애 버리므로 실제 잔차는 0 이 되지만, 없앤 게 아니라 직선 쪽으로
    // 떠넘긴 것이라 원래 차이를 그대로 들고 있어야 판단이 된다.
    out.joints.push({ node: p.idx[p.idx.length - 1], seg: j, dth, dk, forced: blend });

    // 평균내면 C² 는 무조건 성립하지만 직선이 살짝 휜다. 기본은 끔.
    if (blend) {
      const th = a.th + dth / 2, k = (a.k + b.k) / 2;
      a.th = th; a.k = k; b.th = th; b.k = k;
    }
  }

  // 4) 제어점 6 개. free 만 안에서 여러 개로 쪼개진다.
  parts.forEach((s, j) => {
    const P = s.idx.map((i) => L.pts[i]);
    const [f0, f1] = res[j];

    if (s.type === 'free' && s.inner) {
      const fr = s.inner.map((f, i) =>
        (i === 0 ? f0 : (i === s.inner.length - 1 ? f1 : f)));
      s.ctrl = [];
      for (let i = 0; i + 1 < fr.length; i++) s.ctrl.push(hermiteToBezier(fr[i], fr[i + 1]));
    } else if (s.type === 'arc' && s.O) {
      // 평균 강제를 켜도 이 최적화는 그대로 돌린다. 예전에는 blend 일 때 건너뛰었는데,
      // 그러면 sigma = 현길이 로 떨어져서 원호가 원에서 13 % 부풀고 그만큼 곡률이
      // 올라간다 (R 0.60 경로에서 |k| 1.809 -> 2.043, 한계 초과 10 개). 평균으로
      // 양 끝 (theta, kappa) 가 바뀌었더라도 "원에서 가장 덜 벗어나는 sigma" 는
      // 여전히 쓸 만한 기준이고, 프레임이 안 바뀐 자리에서는 결과가 정확히 같다.
      s.scale = fitArcScale(f0, f1, s.O, s.R);
      s.ctrl = [hermiteToBezier(f0, f1, s.scale)];
    } else {
      s.ctrl = [hermiteToBezier(f0, f1)];
    }

    s.frames = [f0, f1];

    // 가운데 노드를 실제로 지나는지 재본다. arc 는 원호의 근사라 정확히 0 이
    // 아니고, 회전각이 커지면 눈에 띄게 벌어진다. 숨기지 말고 그대로 보여준다.
    s.dev = 0;
    for (let q = 1; q + 1 < P.length; q++) {
      let best = Infinity;
      for (const c of s.ctrl) {
        for (let i = 0; i <= 60; i++) best = Math.min(best, dist(deCasteljau(c, i / 60), P[q]));
      }
      s.dev = Math.max(s.dev, best);
    }

    out.segs.push(s);
  });

  return out;
}

function layerCtrls(L, opts) {
  const r = buildLayer(L, opts);
  const out = [];
  for (const s of r.segs) for (const c of s.ctrl) out.push(c);
  return out;
}

// 노드 배열과 플래그 배열을 같이 움직인다. 따로 놀면 경계가 엉뚱한 데로 간다.
function nodeInsert(L, i, p) {
  L.brk = flagArray(L, 'brk');
  L.link = flagArray(L, 'link');
  L.pts.splice(i, 0, p);
  L.brk.splice(i, 0, false);
  L.link.splice(i, 0, false);
}

function nodeRemove(L, i) {
  L.brk = flagArray(L, 'brk');
  L.link = flagArray(L, 'link');
  L.pts.splice(i, 1);
  L.brk.splice(i, 1);
  L.link.splice(i, 1);
}

function nodeReset(L, pts) {
  L.pts = pts;
  L.brk = pts.map(() => false);
  L.link = pts.map(() => false);
}

function reverseLayer(L) {
  const n = L.pts.length;
  const parts = splitSegments(L) || [];
  const b = brkFlags(L);
  L.pts.reverse();
  L.brk = b.slice().reverse();
  // 전이 표시는 '세그먼트의 시작 노드' 에 달려 있다. 뒤집으면 시작이 끝으로
  // 가므로 반대쪽 노드로 옮겨 준다.
  const nl = new Array(n).fill(false);
  const lk = linkFlags(L);
  for (const s of parts) {
    if (!lk[s.idx[0]]) continue;
    nl[n - 1 - s.idx[s.idx.length - 1]] = true;
  }
  L.link = nl;
}

function lerpPt(a, b, u) {
  return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u];
}

function deCasteljau(ctrl, u) {
  const p = ctrl.map((q) => [q[0], q[1]]);
  for (let r = 0; r < p.length - 1; r++) {
    for (let i = 0; i < p.length - 1 - r; i++) {
      p[i] = [(1 - u) * p[i][0] + u * p[i + 1][0],
              (1 - u) * p[i][1] + u * p[i + 1][1]];
    }
  }
  return p[0];
}

function hodograph(ctrl) {
  const n = ctrl.length - 1;
  const out = [];
  for (let i = 0; i < n; i++) {
    out.push([n * (ctrl[i + 1][0] - ctrl[i][0]), n * (ctrl[i + 1][1] - ctrl[i][1])]);
  }
  return out;
}

const GL10_X = [-0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
                -0.4333953941292472, -0.1488743389816312, 0.1488743389816312,
                0.4333953941292472, 0.6794095682990244, 0.8650633666889845,
                0.9739065285171717];
const GL10_W = [0.0666713443086881, 0.1494513491505806, 0.2190863625159820,
                0.2692667193099963, 0.2955242247147529, 0.2955242247147529,
                0.2692667193099963, 0.2190863625159820, 0.1494513491505806,
                0.0666713443086881];

function segLength(ctrl) {
  const d1 = hodograph(ctrl);
  let total = 0;
  for (let i = 0; i < GL10_X.length; i++) {
    const v = deCasteljau(d1, 0.5 * (GL10_X[i] + 1));
    total += GL10_W[i] * Math.hypot(v[0], v[1]);
  }
  return total * 0.5;
}

function segKappaMax(ctrl, samples = 200) {
  const d1 = hodograph(ctrl);
  const d2 = hodograph(d1);
  let best = 0;
  for (let i = 0; i <= samples; i++) {
    const u = i / samples;
    const v = deCasteljau(d1, u);
    const a = deCasteljau(d2, u);
    const sp = Math.hypot(v[0], v[1]);
    if (sp < 1e-9) continue;
    best = Math.max(best, Math.abs(v[0] * a[1] - v[1] * a[0]) / (sp * sp * sp));
  }
  return best;
}

// 곡선을 호길이 등간격으로 다시 딴다. t 를 그냥 등분하면 급한 구간에서 성겨진다.
function resample(segs, spacing, closed) {
  const dense = [];
  const cum = [0];
  const per = 40;

  for (const ctrl of segs) {
    for (let i = 0; i < per; i++) {
      const p = deCasteljau(ctrl, i / per);
      if (dense.length) cum.push(cum[cum.length - 1] + dist(p, dense[dense.length - 1]));
      dense.push(p);
    }
  }

  if (!dense.length) return [];

  const tail = deCasteljau(segs[segs.length - 1], 1);
  cum.push(cum[cum.length - 1] + dist(tail, dense[dense.length - 1]));
  dense.push(tail);

  const total = cum[cum.length - 1];
  const out = [];
  let j = 0;

  for (let s = 0; s < total - 1e-9; s += spacing) {
    while (j < cum.length - 2 && cum[j + 1] < s) j++;
    const span = cum[j + 1] - cum[j];
    const u = span < 1e-12 ? 0 : (s - cum[j]) / span;
    out.push([dense[j][0] + u * (dense[j + 1][0] - dense[j][0]),
              dense[j][1] + u * (dense[j + 1][1] - dense[j][1])]);
  }

  if (!closed) out.push(tail);
  return out;
}

function signedArea(pts) {
  let a = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], q = pts[(i + 1) % pts.length];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return a / 2;
}

// 중심선을 법선으로 d 만큼 민다. outward=true 면 면적이 커지는 쪽으로.
function offsetPolyline(pts, closed, d, outward) {
  const frames = knotFrames(pts, closed);
  if (frames.length < 2) return [];

  const push = (sign) => frames.map((f) => [
    f.p[0] + sign * d * -Math.sin(f.th),
    f.p[1] + sign * d * Math.cos(f.th),
  ]);

  const plus = push(+1);
  const minus = push(-1);

  if (!closed) return outward ? plus : minus;

  const bigger = Math.abs(signedArea(plus)) >= Math.abs(signedArea(minus)) ? plus : minus;
  const smaller = bigger === plus ? minus : plus;
  return outward ? bigger : smaller;
}

// ---------------------------------------------------------------- 좌표 변환

function worldToImage(w) {
  return [(w[0] - S.map.ox) / S.map.res, S.map.h - (w[1] - S.map.oy) / S.map.res];
}

function imageToWorld(p) {
  return [S.map.ox + p[0] * S.map.res, S.map.oy + (S.map.h - p[1]) * S.map.res];
}

function worldToScreen(w) {
  const i = worldToImage(w);
  return [i[0] * S.view.scale + S.view.tx, i[1] * S.view.scale + S.view.ty];
}

function screenToWorld(s) {
  return imageToWorld([(s[0] - S.view.tx) / S.view.scale, (s[1] - S.view.ty) / S.view.scale]);
}

// ---------------------------------------------------------------- 지도 읽기

function parsePGM(buf) {
  const bytes = new Uint8Array(buf);
  let pos = 0;

  const token = () => {
    while (pos < bytes.length) {
      const c = bytes[pos];
      if (c === 35) { while (pos < bytes.length && bytes[pos] !== 10) pos++; }
      else if (c === 32 || c === 9 || c === 10 || c === 13) pos++;
      else break;
    }
    const start = pos;
    while (pos < bytes.length && ![32, 9, 10, 13].includes(bytes[pos])) pos++;
    return String.fromCharCode(...bytes.slice(start, pos));
  };

  const magic = token();
  if (magic !== 'P5') throw new Error(`P5 pgm 이 아니다 (${magic})`);

  const w = parseInt(token(), 10);
  const h = parseInt(token(), 10);
  const maxval = parseInt(token(), 10);
  pos++;   // maxval 뒤 공백 한 개

  const data = bytes.subarray(pos, pos + w * h);
  return { w, h, maxval, data };
}

function parseMapYaml(text) {
  const num = (key) => {
    const m = text.match(new RegExp(`^\\s*${key}\\s*:\\s*([-\\d.eE+]+)`, 'm'));
    return m ? parseFloat(m[1]) : null;
  };
  const org = text.match(/^\s*origin\s*:\s*\[([^\]]+)\]/m);
  const o = org ? org[1].split(',').map((v) => parseFloat(v)) : [0, 0, 0];
  return { res: num('resolution'), ox: o[0], oy: o[1] };
}

function buildMapCanvas(pgm) {
  const off = document.createElement('canvas');
  off.width = pgm.w; off.height = pgm.h;
  const octx = off.getContext('2d');
  const img = octx.createImageData(pgm.w, pgm.h);

  // 임계값은 kau_localization/scripts/save_map.py 와 같게 둔다. 편집기에서 보이는
  // 벽이 지도 품질 검사가 세는 벽과 달라지면 안 된다.
  for (let i = 0; i < pgm.w * pgm.h; i++) {
    const v = pgm.data[i];
    let r, g, b;
    if (v < 100) { r = 40; g = 44; b = 52; }           // 벽
    else if (v > 250) { r = 246; g = 247; b = 250; }   // 빈 공간
    else { r = 150; g = 155; b = 165; }                // 미탐색
    img.data[i * 4] = r; img.data[i * 4 + 1] = g;
    img.data[i * 4 + 2] = b; img.data[i * 4 + 3] = 255;
  }

  octx.putImageData(img, 0, 0);
  return off;
}

async function loadMapFromServer() {
  const [pgmRes, yamlRes] = await Promise.all([
    fetch(`${MAP_DIR}${MAP_NAME}.pgm`),
    fetch(`${MAP_DIR}${MAP_NAME}.yaml`),
  ]);
  if (!pgmRes.ok || !yamlRes.ok) throw new Error('지도 파일을 못 읽었다');
  setMap(MAP_NAME, await pgmRes.arrayBuffer(), await yamlRes.text());
}

let pendingPgm = null, pendingYaml = null;

// 벽 픽셀만의 경계상자. 참조 이미지를 처음 놓을 때 pgm 캔버스 전체가 아니라
// 여기에 맞춘다 — pgm 은 주변 여백을 넉넉히 물고 있어서 그대로 맞추면 이미지가
// 실제 방보다 훨씬 크게 깔린다.
function wallBBox(pgm, meta) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;

  for (let iy = 0; iy < pgm.h; iy++) {
    for (let ix = 0; ix < pgm.w; ix++) {
      if (pgm.data[iy * pgm.w + ix] >= 100) continue;   // 벽만
      const wx = meta.ox + ix * meta.res;
      const wy = meta.oy + (pgm.h - iy) * meta.res;
      if (wx < x0) x0 = wx;
      if (wx > x1) x1 = wx;
      if (wy < y0) y0 = wy;
      if (wy > y1) y1 = wy;
    }
  }

  if (!isFinite(x0)) return null;
  return { x0, y0, x1, y1 };
}

function setMap(name, pgmBuf, yamlText) {
  const pgm = parsePGM(pgmBuf);
  const meta = parseMapYaml(yamlText);
  if (!meta.res) throw new Error('yaml 에 resolution 이 없다');

  S.map = {
    name, w: pgm.w, h: pgm.h, res: meta.res, ox: meta.ox, oy: meta.oy,
    canvas: buildMapCanvas(pgm),
    wall: wallBBox(pgm, meta),
  };

  document.getElementById('mapinfo').innerHTML =
    `<b>${name}</b>  ${pgm.w}x${pgm.h} px · ${meta.res} m/px<br>` +
    `origin [${meta.ox.toFixed(3)}, ${meta.oy.toFixed(3)}]  ` +
    `= ${(pgm.w * meta.res).toFixed(2)} x ${(pgm.h * meta.res).toFixed(2)} m` +
    (S.map.wall
      ? `<br>벽 범위 x [${S.map.wall.x0.toFixed(2)}, ${S.map.wall.x1.toFixed(2)}] · ` +
        `y [${S.map.wall.y0.toFixed(2)}, ${S.map.wall.y1.toFixed(2)}]  ` +
        `= ${(S.map.wall.x1 - S.map.wall.x0).toFixed(2)} x ` +
        `${(S.map.wall.y1 - S.map.wall.y0).toFixed(2)} m`
      : '');

  fitView();
}

// ---------------------------------------------------------------- 궤적 CSV

function parseCSV(text) {
  const lines = text.split(/\r?\n/).filter((l) => l.trim() && !l.trim().startsWith('#'));
  if (!lines.length) return [];

  const head = lines[0].split(',').map((s) => s.trim().toLowerCase());
  let xi = head.indexOf('x'), yi = head.indexOf('y'), start = 1;

  if (xi < 0 || yi < 0) { xi = 1; yi = 2; start = 0; }   // 헤더 없음: t,x,y,yaw 로 본다

  const out = [];
  for (let i = start; i < lines.length; i++) {
    const c = lines[i].split(',');
    const x = parseFloat(c[xi]), y = parseFloat(c[yi]);
    if (Number.isFinite(x) && Number.isFinite(y)) out.push([x, y]);
  }
  return out;
}

// 여러 바퀴가 한 파일에 들어 있는 옛 형식을 바퀴 단위로 자른다.
//
// 지금 record_trajectory.py 는 바퀴마다 파일을 따로 쓰므로 새 데이터에는 안
// 쓰인다. 이미 딴 궤적을 계속 읽을 수 있게 남겨 둔다.
function splitLaps(pts, radius = 0.40, minLapLength = 2.0) {
  if (!pts || pts.length < 10) return pts && pts.length ? [pts] : [];

  const p0 = pts[0];
  const marks = [0];
  let run = null;
  let acc = 0;

  for (let i = 1; i < pts.length; i++) {
    acc += dist(pts[i], pts[i - 1]);
    const d = dist(pts[i], p0);

    if (d < radius && acc > minLapLength) {
      if (!run || d < run.d) run = { index: i, d };
    } else if (run) {
      marks.push(run.index);
      run = null;
      acc = 0;
    }
  }

  if (run) marks.push(run.index);
  if (marks[marks.length - 1] !== pts.length - 1) marks.push(pts.length - 1);

  const laps = [];
  for (let i = 0; i < marks.length - 1; i++) {
    const lap = pts.slice(marks[i], marks[i + 1] + 1);
    if (lap.length >= 10) laps.push(lap);
  }

  return laps.length ? laps : [pts];
}

// 한 바퀴를 anchor 최근접점에서 출발하도록 돌린 뒤 호길이 등간격 n 점.
//
// 바퀴마다 잘린 위상이 다르면(다른 세션, 파일을 지웠을 때) 같은 k 번째 표본이
// 트랙의 다른 자리를 가리킨다. anchor 로 맞춰서 없앤다.
function resampleClosedFrom(lap, anchor, n = 400) {
  if (!lap || lap.length < 2) return [];

  let m = 0, md = Infinity;
  lap.forEach((p, i) => { const d = dist(p, anchor); if (d < md) { md = d; m = i; } });

  const rot = lap.slice(m).concat(lap.slice(0, m));
  rot.push(rot[0]);                       // 폐곡선으로 닫는다

  const cum = [0];
  for (let i = 1; i < rot.length; i++) cum.push(cum[i - 1] + dist(rot[i], rot[i - 1]));

  const total = cum[cum.length - 1];
  if (total < 1e-9) return [];

  const out = [];
  let j = 0;

  for (let k = 0; k < n; k++) {
    const s = total * k / n;
    while (j < cum.length - 2 && cum[j + 1] < s) j++;
    const span = cum[j + 1] - cum[j];
    const u = span < 1e-12 ? 0 : (s - cum[j]) / span;
    out.push([rot[j][0] + u * (rot[j + 1][0] - rot[j][0]),
              rot[j][1] + u * (rot[j + 1][1] - rot[j][1])]);
  }

  return out;
}

function closestOnSegment(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const l2 = dx * dx + dy * dy;
  if (l2 < 1e-18) return { q: a, d: dist(p, a) };
  const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2));
  const q = [a[0] + t * dx, a[1] + t * dy];
  return { q, d: dist(p, q) };
}

// index k 주변 +-w 구간 중 p 에 가장 가까운 점. 전역 최근접을 쓰면 헤어핀처럼
// 트랙이 자기 자신과 가까워지는 곳에서 반대편 구간에 붙는다.
function nearestWindowed(p, samples, k, w) {
  const n = samples.length;
  let best = null, bd = Infinity;
  for (let off = -w; off <= w; off++) {
    const i = ((k + off) % n + n) % n;
    const { q, d } = closestOnSegment(p, samples[i], samples[(i + 1) % n]);
    if (d < bd) { bd = d; best = q; }
  }
  return best;
}

// 바퀴들을 겹쳐서 평균 곡선과 벌어짐을 낸다.
//
// 대응은 호길이 비율이 아니라 **최근접점**으로 잡는다. 바퀴마다 주행거리가
// 조금씩 다르면 호길이 비율은 같은 k 번째 표본을 트랙의 다른 자리에 놓고,
// 그 세로 어긋남이 가로 오차로 둔갑해서 측위를 실제보다 나쁘게 보이게 한다
// (무노이즈 합성 궤적에서 1.8 cm 를 쟀다. 정답은 0 이다).
//
// 벌어짐이 측위 반복정밀도이고 global path 정확도의 하한이다 (README 6.1).
// record_trajectory.py 의 align_laps 와 같은 알고리즘이다.
function alignLaps(laps, n = 400) {
  const use = laps.filter((l) => l && l.length >= 2);
  if (!use.length) return { pts: [], spread: 0 };
  if (use.length === 1) return { pts: use[0].slice(), spread: 0 };

  const anchor = use[0][0];
  const sampled = use.map((l) => resampleClosedFrom(l, anchor, n)).filter((x) => x.length);
  if (sampled.length < 2) return { pts: sampled[0] || [], spread: 0 };

  const ref = sampled[0];
  const w = Math.max(4, Math.floor(n / 10));
  const pts = [];
  let spread = 0;

  for (let k = 0; k < n; k++) {
    const group = [ref[k]];
    for (let i = 1; i < sampled.length; i++) {
      group.push(nearestWindowed(ref[k], sampled[i], k, w));
    }

    const m = [group.reduce((a, q) => a + q[0], 0) / group.length,
               group.reduce((a, q) => a + q[1], 0) / group.length];
    pts.push(m);
    for (const q of group) spread = Math.max(spread, dist(q, m));
  }

  return { pts, spread };
}

const MAX_LAP_FILES = 40;

// `inner_1.csv` `inner_2.csv` ... 를 훑는다.
//
// 번호가 비어 있어도 된다 — 잘못 돈 바퀴를 지우면 구멍이 난다. 그래서 첫
// 404 에서 멈추지 않고 정해진 범위를 전부 두드린다. 디렉터리 목록을 파싱하지
// 않으므로 서버 종류를 안 탄다.
async function loadTrajectories() {
  for (const lane of ['inner', 'outer']) {
    const probes = [];
    for (let i = 1; i <= MAX_LAP_FILES; i++) {
      probes.push(fetch(`${DATA_DIR}${lane}_${i}.csv`)
        .then((r) => (r.ok ? r.text().then((t) => ({ name: `${lane}_${i}.csv`, pts: parseCSV(t) })) : null))
        .catch(() => null));
    }

    let laps = (await Promise.all(probes)).filter((l) => l && l.pts.length >= 10);

    // 옛 형식: 한 파일에 여러 바퀴가 들어 있으면 잘라서 쓴다.
    if (!laps.length) {
      try {
        const res = await fetch(`${DATA_DIR}${lane}.csv`);
        if (res.ok) {
          const pts = parseCSV(await res.text());
          laps = splitLaps(pts).map((l, i) => ({ name: `${lane}.csv #${i + 1}`, pts: l }));
        }
      } catch (e) { /* 아직 없을 수 있다 */ }
    }

    if (laps.length) setLaps(lane, laps);
  }
  updateTrajInfo();
}

function setLaps(lane, laps) {
  S.laps[lane] = laps;
  S.lapAvg[lane] = alignLaps(laps.map((l) => l.pts));
  S.lapSel[lane] = laps.length > 1 ? 'avg' : 0;
}

function addLap(lane, name, pts) {
  const laps = S.laps[lane].filter((l) => l.name !== name);
  laps.push({ name, pts });
  laps.sort((a, b) => a.name.localeCompare(b.name, undefined, { numeric: true }));
  setLaps(lane, laps);
}

function lapLength(pts) {
  let d = 0;
  for (let i = 1; i < pts.length; i++) d += dist(pts[i], pts[i - 1]);
  return d;
}

function updateTrajInfo() {
  const el = document.getElementById('trajinfo');
  const parts = [];

  for (const lane of ['inner', 'outer']) {
    const laps = S.laps[lane];
    if (!laps.length) {
      parts.push(`<span class="bad">${lane}_1.csv ... 없음</span>`);
      continue;
    }

    const line = [`<b>${lane}</b> ${laps.length} 바퀴`];

    if (laps.length > 1) {
      const { spread } = S.lapAvg[lane];
      // 이 값보다 정밀하게 lane 을 그려도 의미가 없다 (README 6.1).
      const cls = spread > 0.15 ? 'bad' : 'ok';
      line.push(`바퀴 간 벌어짐 <span class="${cls}">${(spread * 100).toFixed(1)} cm</span>`);

      const lens = laps.map((l) => lapLength(l.pts)).sort((a, b) => a - b);
      const mid = lens[lens.length >> 1];
      const odd = laps.filter((l) => Math.abs(lapLength(l.pts) - mid) > 0.10 * mid);
      if (odd.length) {
        line.push(`<span class="bad">길이가 튀는 바퀴: ${odd.map((l) => l.name).join(', ')}</span>`);
      }
    }

    parts.push(line.join('<br>&nbsp;&nbsp;'));
  }

  el.innerHTML = parts.join('<br>');
  renderLapPicker();
}

// 씨앗으로 어느 바퀴를 쓸지 고르는 드롭다운. 활성 레이어의 lane 을 따라간다.
function renderLapPicker() {
  const sel = document.getElementById('lapPick');
  const def = LAYER_DEFS.find((d) => d.id === S.active);

  if (!def || def.kind !== 'lane' || !S.laps[def.lane].length) {
    sel.innerHTML = '<option>—</option>';
    sel.disabled = true;
    return;
  }

  const laps = S.laps[def.lane];
  const opts = [];

  if (laps.length > 1) opts.push(['avg', `평균 (${laps.length} 바퀴)`]);
  laps.forEach((l, i) => opts.push([String(i), `${l.name} (${lapLength(l.pts).toFixed(1)} m)`]));

  sel.disabled = false;
  sel.innerHTML = opts.map(([v, t]) =>
    `<option value="${v}" ${String(S.lapSel[def.lane]) === v ? 'selected' : ''}>${t}</option>`).join('');
}

// 씨앗으로 쓸 점열. 사용자가 고른 바퀴(또는 평균)를 돌려준다.
function seedSource(lane) {
  const laps = S.laps[lane];
  if (!laps || !laps.length) return null;
  const sel = S.lapSel[lane];
  if (sel === 'avg') return (S.lapAvg[lane] || alignLaps(laps.map((l) => l.pts))).pts;
  return (laps[sel] || laps[0]).pts;
}

// 궤적을 호길이 등간격으로 줄여서 노드 씨앗을 만든다.
function seedFromTrajectory(traj, spacing) {
  if (!traj || traj.length < 2) return [];
  const out = [traj[0]];
  let acc = 0;
  for (let i = 1; i < traj.length; i++) {
    acc += dist(traj[i], traj[i - 1]);
    if (acc >= spacing) { out.push(traj[i]); acc = 0; }
  }
  // 폐곡선이라 마지막 점이 첫 점에 너무 붙으면 접선이 튄다
  if (out.length > 2 && dist(out[out.length - 1], out[0]) < spacing * 0.5) out.pop();
  return out;
}

// 점 p 에서 폴리라인까지의 최단거리. 샘플점까지의 거리가 아니라 선분까지 잰다.
function distToPolyline(p, poly) {
  let best = Infinity;
  for (let i = 1; i < poly.length; i++) {
    const a = poly[i - 1], b = poly[i];
    const vx = b[0] - a[0], vy = b[1] - a[1];
    const L2 = vx * vx + vy * vy;
    let t = L2 < 1e-18 ? 0 : ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    best = Math.min(best, Math.hypot(p[0] - (a[0] + t * vx), p[1] - (a[1] + t * vy)));
  }
  return best;
}

// segment 하나가 자기 구간의 궤적에서 얼마나 벗어났는지. {dev, at} 을 준다.
// at 은 가장 크게 벗어난 궤적 점의 인덱스다 — 노드를 끼울 자리.
function segDeviation(ctrl, traj, from, to, samples = 40) {
  const poly = [];
  for (let i = 0; i <= samples; i++) poly.push(deCasteljau(ctrl, i / samples));

  let dev = -1, at = -1;
  const n = traj.length;
  for (let k = from + 1; ; k++) {
    const i = k % n;
    if (i === to) break;
    const d = distToPolyline(traj[i], poly);
    if (d > dev) { dev = d; at = i; }
    if (k - from > n) break;          // 안전장치
  }
  return { dev, at };
}

// 궤적과 발행될 곡선 사이의 오차가 tol 이하가 되도록 노드를 적응적으로 깐다.
//
// 등간격(seedFromTrajectory)과 달리 간격을 사람이 정하지 않는다. 성긴 노드에서
// 시작해 "궤적에서 가장 많이 벗어난 segment" 에 그 최악점을 노드로 끼우기를
// 오차가 tol 아래로 내려갈 때까지 반복한다. 직진은 한 segment 로 덮여도 오차가
// 안 생기니 성기게 남고, 코너는 계속 쪼개져 촘촘해진다.
//
// 노드를 하나 끼우면 이웃 노드의 (theta, kappa) 가 바뀌어 주변 segment 모양도
// 달라지므로 매 회 전체를 다시 계산한다 (README 4.4 의 C2 공유 때문).
//
// {pts, dev, blocked} 를 준다. blocked 는 tol 에 도달하기 전에 최소 간격(또는
// 노드 상한)에 걸려 멈췄다는 뜻이다. 이때 dev 가 실제로 달성한 오차다 —
// 조용히 포기하면 "오차 1 cm 로 깔았다" 고 착각하게 되므로 반드시 알린다.
function seedAdaptive(traj, tol, closed, opts = {}) {
  if (!traj || traj.length < 4) return { pts: [], dev: 0, blocked: false };

  const minSpacing = opts.minSpacing ?? 0.15;
  const maxNodes = opts.maxNodes ?? 300;
  const n = traj.length;

  // 시작 노드. 너무 성기면 첫 곡선이 엉뚱하게 말리므로 8 개부터 간다.
  const start = Math.min(opts.start ?? 8, n);
  const idx = [];
  for (let i = 0; i < start; i++) idx.push(Math.round(i * n / start) % n);
  if (!closed) { idx[0] = 0; idx[idx.length - 1] = n - 1; }

  let dev = 0, blocked = false;

  for (let guard = 0; guard < maxNodes; guard++) {
    const nodes = idx.map((i) => traj[i]);
    const segs = buildSegments(nodes, closed);
    if (!segs.length) break;

    const cand = segs.map((ctrl, si) => {
      const from = idx[si];
      const to = idx[(si + 1) % idx.length];
      return { si, ...segDeviation(ctrl, traj, from, to) };
    }).filter((c) => c.at >= 0).sort((a, b) => b.dev - a.dev);

    dev = cand.length ? cand[0].dev : 0;
    if (!cand.length || dev <= tol) break;
    if (idx.length >= maxNodes) { blocked = true; break; }

    // 최악부터 훑되, 끼워 넣으면 이웃과 너무 붙는 자리는 건너뛴다.
    // 측위 지터를 노드로 만들지 않으려는 것이다.
    let put = -1;
    for (const c of cand) {
      if (c.dev <= tol) break;
      const p = traj[c.at];
      const a = traj[idx[c.si]], b = traj[idx[(c.si + 1) % idx.length]];
      if (dist(p, a) < minSpacing || dist(p, b) < minSpacing) continue;
      put = c.at;
      break;
    }
    if (put < 0) { blocked = true; break; }

    idx.push(put);
    idx.sort((a, b) => a - b);
  }

  return { pts: idx.map((i) => traj[i]), dev, blocked };
}

// ---------------------------------------------------------------- undo

function snapshot() {
  return JSON.stringify({
    layers: S.layers,
    active: S.active,
  });
}

function pushUndo() {
  S.undo.push(snapshot());
  if (S.undo.length > 200) S.undo.shift();
  S.redo.length = 0;
}

// 저장본을 현재 레이어 구성에 **맞춰 넣는다.** 통째로 대입하면 안 된다 —
// 나중에 레이어를 추가했을 때 그 레이어가 아예 없어져서 그리기부터 터진다.
// (2026-08-21 에 center 레이어를 넣으면서 실제로 그렇게 됐다.)
function restore(snap) {
  const o = JSON.parse(snap);
  let src = o.layers || {};
  let moved = false;

  // 예전 저장본에는 center 가 없고 주행면 중심선이 lane_inner 에 들어 있었다.
  // 그대로 옮긴다. 양쪽에 남겨 두면 같은 경로가 두 번 내보내진다.
  if (!src.center && src.lane_inner && (src.lane_inner.pts || []).length) {
    src = Object.assign({}, src, { center: src.lane_inner, lane_inner: null });
    moved = true;
  }

  const next = {};
  for (const d of LAYER_DEFS) {
    const L = src[d.id];
    next[d.id] = (L && Array.isArray(L.pts))
      ? { pts: L.pts, closed: L.closed !== false, visible: L.visible !== false,
          brk: Array.isArray(L.brk) ? L.brk : [], link: Array.isArray(L.link) ? L.link : [] }
      : { pts: [], closed: true, visible: true, brk: [], link: [] };
  }

  S.layers = next;
  // 옮겨 왔는데 활성 레이어가 그대로면 빈 lane_inner 를 보게 된다 — 노드가
  // 사라진 것처럼 보이는 게 딱 이 경우다.
  S.active = (moved && o.active === 'lane_inner') ? 'center'
    : (next[o.active] ? o.active : LAYER_DEFS[0].id);
  renderLayers();
  return moved;
}

function undo() {
  if (!S.undo.length) return;
  S.redo.push(snapshot());
  restore(S.undo.pop());
  draw();
}

function redo() {
  if (!S.redo.length) return;
  S.undo.push(snapshot());
  restore(S.redo.pop());
  draw();
}

// ---------------------------------------------------------------- 그리기

function fitView() {
  if (!S.map) return;
  const r = cv.getBoundingClientRect();
  const s = Math.min(r.width / S.map.w, r.height / S.map.h) * 0.92;
  S.view.scale = s;
  S.view.tx = (r.width - S.map.w * s) / 2;
  S.view.ty = (r.height - S.map.h * s) / 2;
  draw();
}

function resize() {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  cv.width = Math.round(r.width * dpr);
  cv.height = Math.round(r.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

// CAD 트랙을 map 프레임으로 옮긴다.
//
//   map_x = ox - sim_y ,  map_y = sim_x - oy      (sim 은 축이 90도 돌아 있다)
//
// 그 뒤에 트랙 중심을 축으로 rot 만큼 더 돌린다. 벽 경계상자로 맞춘 기본값의
// 잔차가 10 cm 안쪽이라 실제로 돌릴 일은 거의 없지만, 재매핑하면 필요해진다.
function trackToMap(p) {
  const T = S.track;
  let x = T.ox - p[1];
  let y = p[0] - T.oy;

  if (T.rot) {
    const sz = (T.data && T.data.world_size_m) || [12, 7];
    const cx = T.ox - sz[1] / 2;
    const cy = sz[0] / 2 - T.oy;
    const c = Math.cos(T.rot), s = Math.sin(T.rot);
    const dx = x - cx, dy = y - cy;
    x = cx + c * dx - s * dy;
    y = cy + s * dx + c * dy;
  }

  return [x, y];
}

// 고리들을 even-odd 로 채운다. 바깥 고리와 안쪽 고리가 같이 오는 도형(주행면,
// 차선 띠)이라 짝수-홀수 규칙이면 구멍이 알아서 뚫린다.
function fillRings(rings, color) {
  if (!rings || !rings.length) return;
  ctx.fillStyle = color;
  ctx.beginPath();
  for (const ring of rings) {
    ring.forEach((p, i) => {
      const q = worldToScreen(trackToMap(p));
      i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]);
    });
    ctx.closePath();
  }
  ctx.fill('evenodd');
}

function drawTrack() {
  const T = S.track;
  if (!T.on || !T.data || !S.map) return;
  const L = T.data.layers || {};

  ctx.save();
  ctx.globalAlpha = T.alpha;
  if (T.road && L.road) fillRings(L.road.rings, 'rgba(120,130,148,.30)');
  if (L.outer_line) fillRings(L.outer_line.rings, '#eef2f8');
  if (L.inner_line) fillRings(L.inner_line.rings, '#eef2f8');
  if (L.center_line) fillRings(L.center_line.rings, '#ffd166');
  if (L.start_line) fillRings(L.start_line.rings, '#ff5f56');

  // 노드 생성의 기준이 될 중심선을 얇게 덧그린다. 어느 선을 따라가는지
  // 눈으로 확인하고 나서 생성하라는 뜻이다.
  const sel = document.getElementById('cadSrc');
  const cen = T.data.centerlines && sel && T.data.centerlines[sel.value];
  if (cen && cen.pts) {
    ctx.strokeStyle = '#54e6c8';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    cen.pts.forEach((p, i) => {
      const q = worldToScreen(trackToMap(p));
      i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]);
    });
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.restore();
}

// 참조 이미지. 지도 바로 위, 격자·궤적·노드보다는 아래에 깐다.
function drawOverlay() {
  const O = S.overlay;
  if (!O.on || !O.img || !S.map) return;

  const pxPerM = S.view.scale / S.map.res;
  const w = O.wm * pxPerM;
  const h = w * (O.img.height / O.img.width);
  const c = worldToScreen([O.cx, O.cy]);

  ctx.save();
  ctx.globalAlpha = O.alpha;
  ctx.translate(c[0], c[1]);
  ctx.rotate(-O.rot);          // 화면 y 는 아래로 증가하니 부호를 뒤집는다
  ctx.drawImage(O.img, -w / 2, -h / 2, w, h);
  ctx.restore();

  // 맞춤 모드일 때만 테두리를 보여준다. 어디까지가 이미지인지 알아야 벽에 맞춘다.
  if (!O.fit) return;
  ctx.save();
  ctx.globalAlpha = 1;
  ctx.translate(c[0], c[1]);
  ctx.rotate(-O.rot);
  ctx.strokeStyle = '#ffd166';
  ctx.setLineDash([6, 4]);
  ctx.lineWidth = 1.5;
  ctx.strokeRect(-w / 2, -h / 2, w, h);
  ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(-8, 0); ctx.lineTo(8, 0);
  ctx.moveTo(0, -8); ctx.lineTo(0, 8); ctx.stroke();
  ctx.restore();
}

function drawGrid() {
  if (!S.show.grid || !S.map) return;
  const r = cv.getBoundingClientRect();
  const tl = screenToWorld([0, 0]);
  const br = screenToWorld([r.width, r.height]);

  ctx.save();
  ctx.lineWidth = 1;
  ctx.font = '10px monospace';

  // 표시 좌표를 켜면 눈금을 그쪽 정수에 맞춘다. map 정수에 그으면 라벨이
  // 3.68 같은 값이 되어 읽을 이유가 없어진다.
  // 맞바꿈이라 map x 선(세로)은 표시 Y 를, map y 선(가로)은 표시 X 를 나타낸다.
  // 축 이름을 안 붙이면 어느 쪽이 X 인지 알 수 없으므로 라벨에 같이 적는다.
  const gx = DISP.on ? DISP.ox : 0;   // 세로선이 지나는 map x 의 기준
  const gy = DISP.on ? DISP.oy : 0;   // 가로선이 지나는 map y 의 기준

  for (let n = Math.floor(tl[0] - gx); n <= Math.ceil(br[0] - gx); n++) {
    const x = gx + n;
    const a = worldToScreen([x, tl[1]]), b = worldToScreen([x, br[1]]);
    ctx.strokeStyle = n === 0 ? 'rgba(255,90,90,.55)' : 'rgba(90,100,120,.28)';
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    ctx.fillStyle = 'rgba(150,160,180,.8)';
    ctx.fillText(DISP.on ? `Y ${n}` : `${n}`, a[0] + 2, r.height - 4);
  }

  for (let n = Math.floor(br[1] - gy); n <= Math.ceil(tl[1] - gy); n++) {
    const y = gy + n;
    const a = worldToScreen([tl[0], y]), b = worldToScreen([br[0], y]);
    ctx.strokeStyle = n === 0 ? 'rgba(255,90,90,.55)' : 'rgba(90,100,120,.28)';
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    ctx.fillStyle = 'rgba(150,160,180,.8)';
    ctx.fillText(DISP.on ? `X ${n}` : `${n}`, 3, a[1] - 2);
  }

  ctx.restore();
}

function drawTrajectory(pts, color) {
  if (!pts || pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  pts.forEach((p, i) => {
    const s = worldToScreen(p);
    i ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]);
  });
  ctx.stroke();
  ctx.restore();
}

// 바퀴가 여럿이면 고른 것만 진하게 그린다. 나머지는 흐리게 남겨서 바퀴끼리
// 얼마나 벌어졌는지 눈으로 보이게 한다 — 그게 측위 반복정밀도다 (README 6.1).
function drawTrajLayer(lane, rgb) {
  const laps = S.laps[lane];
  if (!laps.length) return;

  if (laps.length === 1) { drawTrajectory(laps[0].pts, `rgba(${rgb},.45)`); return; }

  for (const l of laps) drawTrajectory(l.pts, `rgba(${rgb},.16)`);

  const sel = S.lapSel[lane];
  const pick = sel === 'avg' ? (S.lapAvg[lane] || {}).pts : (laps[sel] || {}).pts;
  if (pick) drawTrajectory(pick, `rgba(${rgb},.75)`);
}

function segOpts() {
  const el = document.getElementById('chkBlend');
  return { blend: !!(el && el.checked) };
}

function c2Tol() {
  return {
    th: (parseFloat(document.getElementById('c2Theta').value) || 0) * Math.PI / 180,
    k: parseFloat(document.getElementById('c2Kappa').value) || 0,
  };
}

// C² 를 못 지킨 경계 노드. 화면에서 빨간 고리로 찍고 검사 패널에 숫자를 낸다.
function c2Bad(R) {
  const t = c2Tol();
  const set = new Set();
  for (const j of R.joints) {
    if (j.forced) continue;            // 평균으로 이미 맞춰 놓은 자리다
    if (Math.abs(j.dth) > t.th || Math.abs(j.dk) > t.k) set.add(j.node);
  }
  return set;
}

const SEG_LABEL = { line: '직선', arc: '원호', link: '전이', free: '보간' };

// 세그먼트 하나의 제어점 6 개를 그대로 펼친다. 현 위 비율과 현에서 벗어난
// 거리를 같이 적는다 — 직선이면 비율이 0.2 간격으로 딱 떨어지고 벗어남이 0 이다.
function ctrlRows(sg) {
  const rows = [];
  sg.ctrl.forEach((c, ci) => {
    const A = c[0], Z = c[5];
    const vx = Z[0] - A[0], vy = Z[1] - A[1];
    const ch = Math.hypot(vx, vy) || 1;
    rows.push(`<tr><td></td><td colspan="6" style="color:#ffd166">` +
      `제어점 6 개${sg.ctrl.length > 1 ? ` (${ci + 1}/${sg.ctrl.length})` : ''} — ` +
      `P0·P5 는 세그먼트 경계</td></tr>`);
    c.forEach((q, i) => {
      const t = ((q[0] - A[0]) * vx + (q[1] - A[1]) * vy) / (ch * ch);
      const off = Math.abs(vx * (A[1] - q[1]) - (A[0] - q[0]) * vy) / ch;
      const d = DISP.on ? toDisp(q) : q;
      rows.push(`<tr><td></td><td colspan="6" style="color:#8b94a4">` +
        `&nbsp;&nbsp;P${i} (${d[0].toFixed(3)}, ${d[1].toFixed(3)}) ` +
        `· 현 ${t.toFixed(3)} · 벗어남 ${(off * 1000).toFixed(1)} mm</td></tr>`);
    });
  });
  return rows.join('');
}

function drawLayer(def) {
  const L = S.layers[def.id];
  if (!L || !L.visible || !L.pts.length) return;

  const isActive = def.id === S.active;
  const R = buildLayer(L, segOpts());
  const limit = curvLimit();
  const brk = brkFlags(L);
  const bad = c2Bad(R);

  // S.show.curve 가 켜져 있으면 실제로 발행될 quintic Bezier 를, 꺼져 있으면
  // 노드를 그대로 이은 폴리라인을 그린다. 어느 쪽이든 저장·내보내기되는 데이터는
  // 같다. 색(곡률 경고)은 두 모드 모두 Bezier 기준이다 — 폴리라인으로 봐도
  // 발행될 곡선이 한계를 넘는지는 계속 보여야 하기 때문이다.
  // 전이(link) 세그먼트만 점선으로 구분한다.
  ctx.save();
  ctx.lineWidth = isActive ? 3 : 2;
  ctx.lineCap = 'round';
  ctx.globalAlpha = isActive ? 1 : 0.55;

  for (const sg of R.segs) {
    let hot = false;
    for (const c of sg.ctrl) if (segKappaMax(c, 40) > limit) hot = true;
    ctx.strokeStyle = hot ? '#ff5f56' : (sg.type === 'link' ? '#9be3ff' : def.color);
    ctx.setLineDash(sg.type === 'link' ? [7, 4] : []);
    ctx.beginPath();
    if (S.show.curve) {
      sg.ctrl.forEach((c, ci) => {
        for (let i = 0; i <= 24; i++) {
          const p = worldToScreen(deCasteljau(c, i / 24));
          (ci === 0 && i === 0) ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]);
        }
      });
    } else {
      sg.idx.forEach((ni, i) => {
        const p = worldToScreen(L.pts[ni]);
        i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]);
      });
    }
    ctx.stroke();
  }

  // 폐곡선이 꺼진 채로 고리를 그리고 있으면, 안 이어진 자리를 빨간 점선으로
  // 보여 준다. 아무 표시가 없으면 "노드는 다 찍었는데 왜 안 붙지" 가 된다.
  if (!L.closed && L.pts.length >= 3) {
    const a = worldToScreen(L.pts[L.pts.length - 1]);
    const b = worldToScreen(L.pts[0]);
    ctx.strokeStyle = 'rgba(255,95,86,.85)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.setLineDash([]);
  ctx.restore();

  // 제어점. 종류가 뭐든 (직선·원호·전이·보간) 발행되는 것은 전부 5차 Bezier 고
  // 조각마다 제어점이 정확히 6 개다. P0/P5 는 세그먼트 경계와 같은 점이고
  // P1~P4 는 곡선 위에 있지 않다 — 직선일 때만 우연히 현 위에 놓인다.
  if (S.show.ctrl && isActive) {
    ctx.save();
    for (const sg of R.segs) {
      for (const c of sg.ctrl) {
        ctx.strokeStyle = 'rgba(255,209,102,.55)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        c.forEach((q, i) => {
          const t = worldToScreen(q);
          i ? ctx.lineTo(t[0], t[1]) : ctx.moveTo(t[0], t[1]);
        });
        ctx.stroke();
        ctx.setLineDash([]);
        c.forEach((q, i) => {
          const t = worldToScreen(q);
          const edge = (i === 0 || i === 5);
          ctx.beginPath();
          ctx.rect(t[0] - 2.5, t[1] - 2.5, 5, 5);
          ctx.fillStyle = edge ? '#ffd166' : '#14171c';
          ctx.fill();
          ctx.strokeStyle = '#ffd166';
          ctx.lineWidth = 1;
          ctx.stroke();
        });
      }
    }
    ctx.restore();
  }

  // 노드. 세그먼트 경계는 사각형, 안쪽 노드는 동그라미다.
  ctx.save();
  L.pts.forEach((p, i) => {
    const sp = worldToScreen(p);
    const hovered = S.hover && S.hover.layer === def.id && S.hover.index === i;
    const isBrk = !R.legacy && brk[i];
    const r = (hovered ? 6 : (isActive ? 4 : 3)) * (isBrk ? 1.15 : 1);
    ctx.globalAlpha = isActive ? 1 : 0.5;
    ctx.fillStyle = i === 0 ? '#ffffff' : def.color;
    ctx.beginPath();
    if (isBrk) ctx.rect(sp[0] - r, sp[1] - r, 2 * r, 2 * r);
    else ctx.arc(sp[0], sp[1], r, 0, Math.PI * 2);
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#14171c';
    ctx.stroke();
    if (bad.has(i)) {
      ctx.beginPath();
      ctx.arc(sp[0], sp[1], r + 4, 0, Math.PI * 2);
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ff5f56';
      ctx.stroke();
    }
  });
  ctx.restore();

  // 진행 방향 화살표 (첫 구간)
  if (isActive && R.segs.length) {
    const sg = R.segs[0];
    const q = L.pts[sg.idx[1]];
    const a = S.show.curve ? deCasteljau(sg.ctrl[0], 0.35) : lerpPt(L.pts[sg.idx[0]], q, 0.35);
    const b = S.show.curve ? deCasteljau(sg.ctrl[0], 0.55) : lerpPt(L.pts[sg.idx[0]], q, 0.55);
    const sa = worldToScreen(a), sb = worldToScreen(b);
    const th = Math.atan2(sb[1] - sa[1], sb[0] - sa[0]);
    ctx.save();
    ctx.translate(sb[0], sb[1]); ctx.rotate(th);
    ctx.fillStyle = def.color;
    ctx.beginPath();
    ctx.moveTo(0, 0); ctx.lineTo(-9, -4.5); ctx.lineTo(-9, 4.5);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }
}

function draw() {
  const r = cv.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);
  ctx.fillStyle = '#14171c';
  ctx.fillRect(0, 0, r.width, r.height);

  if (!S.map) {
    ctx.fillStyle = '#8b94a4';
    ctx.font = '13px monospace';
    ctx.fillText('지도를 못 읽었다. pgm 과 yaml 을 창에 끌어다 놓거나,', 20, 40);
    ctx.fillText('워크스페이스 루트에서 python3 -m http.server 로 띄울 것.', 20, 60);
    return;
  }

  if (S.show.map) {
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.map.canvas, S.view.tx, S.view.ty,
                  S.map.w * S.view.scale, S.map.h * S.view.scale);
    ctx.restore();
  }

  drawOverlay();
  drawTrack();
  drawGrid();

  if (S.show.traj) {
    drawTrajLayer('inner', '78,163,255');
    drawTrajLayer('outer', '255,178,78');
  }

  for (const d of LAYER_DEFS) if (d.id !== S.active) drawLayer(d);
  for (const d of LAYER_DEFS) if (d.id === S.active) drawLayer(d);

  updateHud();
}

function updateHud() {
  const L = S.layers[S.active];
  const def = LAYER_DEFS.find((d) => d.id === S.active);
  const R = buildLayer(L, segOpts());
  const segs = [];
  for (const sg of R.segs) for (const c of sg.ctrl) segs.push(c);

  let total = 0, kmax = 0;
  for (const c of segs) { total += segLength(c); kmax = Math.max(kmax, segKappaMax(c, 40)); }

  const nBad = c2Bad(R).size;
  const forced = R.joints.filter((j) => j.forced &&
    (Math.abs(j.dth) > c2Tol().th || Math.abs(j.dk) > c2Tol().k)).length;
  const c = S.cursor;
  document.getElementById('hud').textContent =
    `${def.label}\n` +
    `노드 ${L.pts.length}  세그먼트 ${R.legacy ? '—' : R.segs.length}` +
    `  Bezier ${segs.length} (제어점 ${segs.length * NCTRL})  길이 ${total.toFixed(2)} m\n` +
    (R.legacy ? '경계 미지정 — 전체를 한 덩어리로 보간 중\n'
              : `C² 위반 경계 ${nBad} / ${R.joints.length}` +
                (forced ? `  (평균 강제 ${forced})` : '') + '\n') +
    `max|k| ${kmax.toFixed(3)} 1/m  (R ${kmax > 1e-6 ? (1 / kmax).toFixed(2) : '∞'} m` +
    `  δ ${steerOfKappa(kmax).toFixed(1)}°)  한계 ${curvLimit().toFixed(3)}` +
    `${kmax > curvLimit() ? '  << 초과' : ''}\n` +
    (c ? (DISP.on
      ? `커서 X ${toDisp(c)[0].toFixed(3)}  Y ${toDisp(c)[1].toFixed(3)} m  [구석 기준]\n` +
        `     map (${c[0].toFixed(3)}, ${c[1].toFixed(3)}) m`
      : `커서 (${c[0].toFixed(3)}, ${c[1].toFixed(3)}) m  [map 기준 — 맞바꿈 꺼짐]`) : '');
}

// ---------------------------------------------------------------- 사이드바

function renderLayers() {
  const box = document.getElementById('layers');
  box.innerHTML = '';

  LAYER_DEFS.forEach((d, i) => {
    const L = S.layers[d.id] || (S.layers[d.id] =
      { pts: [], closed: true, visible: true, brk: [], link: [] });
    const el = document.createElement('div');
    el.className = 'layer' + (d.id === S.active ? ' active' : '');
    el.innerHTML =
      `<input type="checkbox" ${L.visible ? 'checked' : ''} title="표시">` +
      `<span class="swatch" style="background:${d.color}"></span>` +
      `<span class="name">${i + 1}. ${d.label}</span>` +
      `<span class="count">${L.pts.length}</span>`;

    el.querySelector('input').onchange = (e) => {
      L.visible = e.target.checked; draw();
      e.stopPropagation();
    };
    el.onclick = (e) => {
      if (e.target.tagName === 'INPUT') return;
      S.active = d.id; renderLayers(); draw();
    };
    box.appendChild(el);
  });

  document.getElementById('chkClosed').checked = S.layers[S.active].closed;
  renderLapPicker();
  runCheck();
}

function runCheck() {
  const limit = curvLimit();
  const lines = [];

  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (!L || L.pts.length < 2) continue;

    const R = buildLayer(L, segOpts());
    const segs = [];
    for (const sg of R.segs) for (const c of sg.ctrl) segs.push(c);

    let total = 0, kmax = 0, bad = 0;
    for (const c of segs) {
      total += segLength(c);
      const k = segKappaMax(c);
      kmax = Math.max(kmax, k);
      if (k > limit) bad++;
    }

    const nBad = c2Bad(R).size;
    const cls = (bad || nBad) ? 'bad' : 'ok';
    const kcls = kmax > limit ? 'bad' : 'ok';
    lines.push(
      `<span class="${cls}">${d.label}</span>: ` +
      `${R.legacy ? segs.length + ' seg(경계 미지정)' : R.segs.length + ' 세그먼트'} · ` +
      `${total.toFixed(2)} m · ` +
      `<span class="${kcls}">max|k| ${kmax.toFixed(3)}</span> ` +
      `(δ ${steerOfKappa(kmax).toFixed(1)}° / 한계 ${steerOfKappa(limit).toFixed(1)}° · ` +
      `한계의 ${(kmax / limit * 100).toFixed(0)}%)` +
      (bad ? ` · <span class="bad">곡률초과 ${bad}</span>` : '') +
      (nBad ? ` · <span class="bad">C² 위반 ${nBad}</span>` : '') +
      // 폐곡선이 꺼져 있으면 마지막 노드와 첫 노드 사이가 아예 안 그려지고
      // 발행도 안 된다. 화면만 보면 "왜 안 이어지지" 로 보이므로 못 박아 둔다.
      (!L.closed && L.pts.length >= 3
        ? `<br>&nbsp;&nbsp;<span class="bad">열린 곡선 — n${L.pts.length - 1} 과 n0 이 안 이어진다 ` +
          `(간격 ${dist(L.pts[L.pts.length - 1], L.pts[0]).toFixed(3)} m). ` +
          `폐곡선 체크박스를 켤 것</span>`
        : ''));
  }

  document.getElementById('check').innerHTML =
    lines.join('<br>') || '레이어가 비어 있다.';

  renderSegList();
}

// 현재 레이어의 세그먼트 목록. 종류·길이·곡률·가운데 노드 오차와, 경계마다의
// C² 차이를 그대로 늘어놓는다. 노드 2 개짜리는 여기서 직선 <-> 전이 를 바꾼다.
function renderSegList() {
  const box = document.getElementById('seglist');
  if (!box) return;

  const L = S.layers[S.active];
  const R = buildLayer(L, segOpts());

  if (R.legacy) {
    box.innerHTML = '경계 노드가 없다. <kbd>Ctrl</kbd>+클릭 으로 노드를 ' +
      '세그먼트 경계로 만들면 여기에 목록이 뜬다.';
    return;
  }

  const tol = c2Tol();
  const limit = curvLimit();
  const rows = [];

  R.segs.forEach((sg, j) => {
    let len = 0, kmax = 0;
    for (const c of sg.ctrl) { len += segLength(c); kmax = Math.max(kmax, segKappaMax(c, 60)); }

    const jt = R.joints[j];
    const gap = jt && (Math.abs(jt.dth) > tol.th || Math.abs(jt.dk) > tol.k);
    const bad = gap && !jt.forced;
    const hot = kmax > limit;

    rows.push(
      `<tr data-seg="${j}">` +
      `<td>#${j}</td>` +
      `<td>${SEG_LABEL[sg.type]}<span style="color:#5c6675">${sg.idx.length}</span></td>` +
      `<td>${len.toFixed(2)}m</td>` +
      `<td class="${hot ? 'bad' : ''}">k${kmax.toFixed(2)}</td>` +
      `<td>${sg.dev > 0.001 ? `<span class="bad">Δ${(sg.dev * 100).toFixed(1)}cm</span>` : ''}</td>` +
      `<td><button data-seg="${j}">P6${sg.ctrl.length > 1 ? `x${sg.ctrl.length}` : ''}</button></td>` +
      `<td>${sg.idx.length === 2
        ? `<button data-link="${sg.idx[0]}">${sg.type === 'link' ? '전이해제' : '전이로'}</button>`
        : ''}</td>` +
      `</tr>` +
      (S.segSel === j ? ctrlRows(sg) : '') +
      (jt ? `<tr><td></td><td colspan="6" class="${gap ? 'bad' : 'ok'}">` +
        `└ 경계 n${jt.node}: Δθ ${(jt.dth * 180 / Math.PI).toFixed(2)}° · ` +
        `Δκ ${jt.dk.toFixed(3)} 1/m` +
        (bad ? ' — C² 아님' : (gap ? ' — 평균으로 강제 (직선이 휜다)' : '')) +
        `</td></tr>` : ''));
  });

  box.innerHTML = `<table class="segs">${rows.join('')}</table>`;

  box.querySelectorAll('button[data-seg]').forEach((b) => {
    b.onclick = () => {
      const j = parseInt(b.getAttribute('data-seg'), 10);
      S.segSel = S.segSel === j ? -1 : j;
      renderSegList();
    };
  });

  box.querySelectorAll('button[data-link]').forEach((b) => {
    b.onclick = () => {
      const i = parseInt(b.getAttribute('data-link'), 10);
      pushUndo();
      L.link = linkFlags(L);
      L.link[i] = !L.link[i];
      renderLayers(); draw(); autosave();
    };
  });
}

function toast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.style.opacity = 1;
  clearTimeout(toast._t);
  toast._t = setTimeout(() => { el.style.opacity = 0; }, 2200);
}

// ---------------------------------------------------------------- 마우스

function pick(sx, sy) {
  const L = S.layers[S.active];
  for (let i = 0; i < L.pts.length; i++) {
    const s = worldToScreen(L.pts[i]);
    if (Math.hypot(s[0] - sx, s[1] - sy) < 9) return i;
  }
  return -1;
}

// 가장 가까운 노드쌍을 찾아 그 사이에 노드를 넣는다. 곡선이 아니라 노드
// 폴리라인에 재는 것은, 넣은 뒤 어느 세그먼트가 직선에서 원호로 바뀌는지가
// 노드 순서로 정해지기 때문이다.
function insertIndex(w) {
  const L = S.layers[S.active];
  const n = L.pts.length;
  if (n < 2) return n;

  const last = L.closed ? n : n - 1;
  let best = -1, bestD = Infinity;

  for (let i = 0; i < last; i++) {
    const d = closestOnSegment(w, L.pts[i], L.pts[(i + 1) % n]).d;
    if (d < bestD) { bestD = d; best = i; }
  }

  return best < 0 ? n : best + 1;
}

let panning = null;
let imgDrag = null;

cv.addEventListener('mousedown', (e) => {
  const r = cv.getBoundingClientRect();
  const sx = e.clientX - r.left, sy = e.clientY - r.top;

  if (e.button === 1 || e.button === 2) {
    panning = [sx, sy];
    return;
  }
  if (!S.map) return;

  // 맞춤 모드에서는 좌드래그가 노드가 아니라 이미지를 잡는다. 노드 편집은
  // 잠시 멈춘다 — 벽에 맞추는 중에 노드가 생기면 오히려 방해된다.
  if (S.overlay.fit && S.overlay.img) {
    const w0 = screenToWorld([sx, sy]);
    imgDrag = [w0[0] - S.overlay.cx, w0[1] - S.overlay.cy];
    return;
  }

  const w = screenToWorld([sx, sy]);
  const hit = pick(sx, sy);

  const L = S.layers[S.active];

  if (e.altKey) {
    if (hit >= 0) { pushUndo(); nodeRemove(L, hit); renderLayers(); draw(); autosave(); }
    return;
  }

  // Ctrl+클릭 = 세그먼트 경계 토글. 노드 위가 아니면 아무 일도 안 한다 —
  // 실수로 경계를 찍으려다 노드가 늘어나는 게 제일 성가시다.
  if (e.ctrlKey || e.metaKey) {
    if (hit >= 0) {
      pushUndo();
      L.brk = brkFlags(L);
      L.brk[hit] = !L.brk[hit];
      if (!L.brk[hit]) { L.link = linkFlags(L); L.link[hit] = false; }
      renderLayers(); draw(); autosave();
    }
    return;
  }

  if (hit >= 0) {
    pushUndo();
    S.drag = hit;
    return;
  }
  pushUndo();
  if (e.shiftKey && L.pts.length >= 2) nodeInsert(L, insertIndex(w), w);
  else nodeInsert(L, L.pts.length, w);
  renderLayers();
  draw();
});

window.addEventListener('mousemove', (e) => {
  const r = cv.getBoundingClientRect();
  const sx = e.clientX - r.left, sy = e.clientY - r.top;

  if (panning) {
    S.view.tx += sx - panning[0];
    S.view.ty += sy - panning[1];
    panning = [sx, sy];
    draw();
    return;
  }
  if (!S.map) return;

  S.cursor = screenToWorld([sx, sy]);

  if (imgDrag) {
    S.overlay.cx = S.cursor[0] - imgDrag[0];
    S.overlay.cy = S.cursor[1] - imgDrag[1];
    overlayInputs();
    draw();
    return;
  }

  if (S.drag !== null) {
    S.layers[S.active].pts[S.drag] = S.cursor;
    draw();
    return;
  }

  // 맞춤 모드에서는 노드를 잡지 않는다. 커서 모양도 이동으로 고정한다.
  if (S.overlay.fit && S.overlay.img) {
    cv.style.cursor = 'move';
    if (S.hover) { S.hover = null; draw(); } else updateHud();
    return;
  }

  const hit = pick(sx, sy);
  const next = hit >= 0 ? { layer: S.active, index: hit } : null;
  const changed = JSON.stringify(next) !== JSON.stringify(S.hover);
  S.hover = next;
  cv.style.cursor = hit >= 0 ? 'move' : 'crosshair';
  if (changed) draw(); else updateHud();
});

window.addEventListener('mouseup', () => {
  if (S.drag !== null) { S.drag = null; renderLayers(); autosave(); }
  if (imgDrag) { imgDrag = null; overlaySave(); }
  panning = null;
});

cv.addEventListener('contextmenu', (e) => e.preventDefault());

cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  if (!S.map) return;
  const r = cv.getBoundingClientRect();
  const sx = e.clientX - r.left, sy = e.clientY - r.top;
  const f = Math.exp(-e.deltaY * 0.0015);

  // 맞춤 모드에서는 휠이 화면이 아니라 이미지를 키운다. 커서 아래 점이
  // 제자리에 있게 잡아 두면 벽 모서리에 맞추기가 훨씬 쉽다.
  if (S.overlay.fit && S.overlay.img) {
    const wc = screenToWorld([sx, sy]);
    S.overlay.cx = wc[0] + (S.overlay.cx - wc[0]) * f;
    S.overlay.cy = wc[1] + (S.overlay.cy - wc[1]) * f;
    S.overlay.wm = Math.max(0.05, S.overlay.wm * f);
    overlayInputs(); overlaySave(); draw();
    return;
  }

  const ns = Math.min(400, Math.max(0.5, S.view.scale * f));
  S.view.tx = sx - (sx - S.view.tx) * (ns / S.view.scale);
  S.view.ty = sy - (sy - S.view.ty) * (ns / S.view.scale);
  S.view.scale = ns;
  draw();
}, { passive: false });

window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;

  const k = e.key.toLowerCase();

  if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
  if ((e.ctrlKey || e.metaKey) && k === 'y') { e.preventDefault(); redo(); return; }
  if (k >= '1' && k <= '5') {
    S.active = LAYER_DEFS[parseInt(k, 10) - 1].id; renderLayers(); draw(); return;
  }
  if (k === 'g') { S.show.grid = !S.show.grid; draw(); }
  if (k === 't') { S.show.traj = !S.show.traj; draw(); }
  if (k === 'm') { S.show.map = !S.show.map; draw(); }
  if (k === 'b') { setCurvePreview(!S.show.curve); }
  if (k === 'p') {
    S.show.ctrl = !S.show.ctrl;
    document.getElementById('chkCtrl').checked = S.show.ctrl;
    draw();
  }
  if (k === 'c' && S.hover) {
    pushUndo();
    const L = S.layers[S.active];
    L.brk = brkFlags(L);
    L.brk[S.hover.index] = !L.brk[S.hover.index];
    renderLayers(); draw(); autosave();
  }
  if (k === 'f') { fitView(); }
  if (k === 'i') {
    document.getElementById('chkImgFit').checked = !S.overlay.fit;
    setImgFit(!S.overlay.fit);
  }
  if (k === 'k') { S.track.on = !S.track.on; document.getElementById('chkTrack').checked = S.track.on; trackSave(); draw(); }
  if (k === 'o') { S.overlay.on = !S.overlay.on; document.getElementById('chkImg').checked = S.overlay.on; overlaySave(); draw(); }
  if (k === '[' || k === ']') {
    if (S.overlay.img) {
      S.overlay.rot += (k === '[' ? 1 : -1) * (e.shiftKey ? 5 : 0.5) * Math.PI / 180;
      overlayInputs(); overlaySave(); draw();
    }
  }
  if (k === 'delete' || k === 'backspace') {
    if (S.hover) { pushUndo(); nodeRemove(S.layers[S.active], S.hover.index); S.hover = null; renderLayers(); draw(); }
  }
});

// ---------------------------------------------------------------- 저장 / 내보내기

function download(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

function autosave() {
  try {
    localStorage.setItem('kau_lane_graph', snapshot());
  } catch (e) { /* 용량 초과는 무시 */ }
}

function loadAutosave() {
  const s = localStorage.getItem('kau_lane_graph');
  if (!s) return false;
  try {
    if (restore(s)) {
      toast(`예전 자동저장을 옮겨 왔다: lane_inner -> center ` +
            `(노드 ${S.layers.center.pts.length} 개)`);
    }
    return true;
  } catch (e) {
    console.warn('자동저장 복원 실패', e);
    return false;
  }
}

// 노드·지도 원점처럼 서로 멀리 떨어진 값. 0.1 mm 면 충분하다.
function fmt(v) { return v.toFixed(4); }

// **제어점은 훨씬 촘촘하게 적어야 한다.**
//
// 곡률은 제어점의 2차 차분에서 나온다. 짧은 조각(전이 0.15 m, 심하면 0.024 m)은
// 제어점 간격이 3 cm ~ 5 mm 밖에 안 되는데, 거기서 0.1 mm 로 반올림하면 2차
// 차분이 25 % 씩 흔들린다. 실측으로 kappa 가 최대 0.75 1/m 틀어졌다 —
// 곡선은 멀쩡한데 파일에 적힌 숫자만 망가지는, 알아채기 제일 어려운 종류다.
//
//   소수 4자리 -> kappa 오차 0.754   5자리 -> 0.030   6자리 -> 0.0075   7자리 -> 0.0033
//
// 7 자리(0.1 um)로 적는다. 파일이 몇 KB 커질 뿐이다.
function fmtC(v) { return v.toFixed(7); }

function exportLaneGraph() {
  const width = parseFloat(document.getElementById('laneWidth').value) || 0.4;
  const roadWidth = parseFloat(document.getElementById('roadWidth').value) || 0.707;
  const lines = [];

  lines.push('# kau_global_path lane_graph — lane_editor.html 로 생성');
  lines.push('#');
  lines.push('# 좌표: map 프레임, 미터. KauPath 발행 시 cm 로 환산한다.');
  lines.push('# nodes 는 곡선이 반드시 통과하는 점이다 (Bezier 제어점이 아니다).');
  lines.push('# 제어점은 노드에서 유도한 (theta, kappa) 로 quintic Hermite -> Bezier');
  lines.push('# 기저변환해서 만든다. kau_control/scripts/fake_path.py 와 같은 식.');
  lines.push('');
  lines.push('lane_graph:');
  lines.push('  frame_id: map');
  lines.push(`  map_source: ${S.map ? S.map.name : 'unknown'}`);
  lines.push(`  map_resolution: ${S.map ? S.map.res : 0}`);
  lines.push(`  map_origin: [${S.map ? fmt(S.map.ox) : 0}, ${S.map ? fmt(S.map.oy) : 0}, 0.0]`);
  lines.push('');

  // 노드 id 는 레이어마다 100 번대로 띄운다. 나중에 lane_change 엣지를 손으로
  // 추가할 때 어느 lane 의 노드인지 번호만 보고 알 수 있어야 한다.
  const base = LAYER_BASE;
  const ids = {};

  lines.push('  nodes:');
  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (!L.pts.length) continue;
    ids[d.id] = L.pts.map((_, i) => base[d.id] + i);
    lines.push(`    # ${d.label}`);
    L.pts.forEach((p, i) => {
      lines.push(`    - {id: ${base[d.id] + i}, x: ${fmt(p[0])}, y: ${fmt(p[1])}}`);
    });
  }

  lines.push('');
  lines.push('  edges:');
  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (L.pts.length < 2) continue;
    const n = L.pts.length;
    const last = L.closed ? n : n - 1;
    const tag = d.kind === 'center'
      ? `type: center, width: ${roadWidth.toFixed(3)}, direction: forward`
      : (d.kind === 'lane'
        ? `type: lane, lane: ${d.lane}, width: ${width.toFixed(3)}, direction: forward`
        : `type: boundary, side: ${d.side}`);
    lines.push(`    # ${d.label}`);
    for (let i = 0; i < last; i++) {
      lines.push(`    - {from: ${ids[d.id][i]}, to: ${ids[d.id][(i + 1) % n]}, ${tag}}`);
    }
  }

  lines.push('');
  lines.push('  # lane 사이 이동이 필요해지면 여기에 한 줄씩 추가한다.');
  lines.push('  # 노드를 다시 그릴 필요는 없다.');
  lines.push('  #   - {from: 12, to: 105, type: lane_change}');
  lines.push('');
  lines.push('  routes:');
  lines.push('  # 기본 경로는 center_loop 다. 차로를 둘로 나눠 보지 않는다 (README 4.1.3).');
  for (const d of LAYER_DEFS) {
    if (d.kind !== 'center' && d.kind !== 'lane') continue;
    const L = S.layers[d.id];
    if (L.pts.length < 2) continue;
    const seq = ids[d.id].slice();
    if (L.closed) seq.push(seq[0]);
    lines.push(`    ${d.route}: [${seq.join(', ')}]`);
  }

  lines.push('');
  lines.push('  closed:');
  for (const d of LAYER_DEFS) {
    lines.push(`    ${d.id}: ${S.layers[d.id].closed}`);
  }
  lines.push('');

  // 편집기가 다시 읽어서 세그먼트 구분을 복원하는 데 쓴다. 소비자는 무시해도 된다.
  lines.push('  # 세그먼트 구분 (편집기 복원용).');
  lines.push('  #   breaks: 세그먼트 경계인 노드 id. 경계와 경계 사이가 곡선 한 조각이다.');
  lines.push('  #   links : 전이 세그먼트의 시작 노드 id. 양 끝 (theta,kappa) 를 이웃에서 받는다.');
  lines.push('  segments:');
  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (L.pts.length < 2) continue;
    const b = brkFlags(L), lk = linkFlags(L);
    const bi = [], li = [];
    L.pts.forEach((_, i) => {
      if (b[i]) bi.push(base[d.id] + i);
      if (lk[i]) li.push(base[d.id] + i);
    });
    lines.push(`    ${d.id}: {breaks: [${bi.join(', ')}], links: [${li.join(', ')}]}`);
  }
  lines.push('');

  // 세그먼트마다 quintic Bezier 제어점 6 개. ctrl[0]/ctrl[5] 가 경계 노드다.
  // 소비자(오프라인 CLI, 발행 노드)는 이 값을 그대로 쓰면 된다 — 다시 피팅하면
  // 여기서 맞춰 놓은 C² 가 깨진다.
  const opts = segOpts();
  lines.push('  # 세그먼트별 quintic Bezier 제어점 6 개. ctrl[0]/ctrl[5] = 경계 노드.');
  lines.push('  # 다시 피팅하지 말 것 — 이 값이 곧 발행될 곡선이다 (README 4.4).');
  lines.push(`  # c2_forced: ${opts.blend} (경계에서 theta/kappa 를 평균냈는지)`);
  lines.push('  bezier:');
  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (L.pts.length < 2) continue;
    const R = buildLayer(L, opts);
    lines.push(`    ${d.id}:`);
    R.segs.forEach((sg) => {
      sg.ctrl.forEach((c) => {
        lines.push(`      - {type: ${sg.type}, ` +
          `from: ${base[d.id] + sg.idx[0]}, to: ${base[d.id] + sg.idx[sg.idx.length - 1]}, ` +
          `length: ${segLength(c).toFixed(6)}, kappa_max: ${segKappaMax(c).toFixed(6)},`);
        lines.push(`         ctrl: [${c.map((q) => `[${fmtC(q[0])}, ${fmtC(q[1])}]`).join(', ')}]}`);
      });
    });
  }
  lines.push('');

  // C² 검증 결과를 주석으로 남긴다. 나중에 파일만 보고도 판정할 수 있어야 한다.
  const tol = c2Tol();
  lines.push('  # C2 검증 (경계에서 들어오는 쪽과 나가는 쪽의 theta/kappa 차이)');
  for (const d of LAYER_DEFS) {
    const L = S.layers[d.id];
    if (L.pts.length < 2) continue;
    const R = buildLayer(L, opts);
    if (R.legacy) { lines.push(`  #   ${d.id}: 경계 미지정 — 전체 보간 (C2 자동 성립)`); continue; }
    const bad = R.joints.filter((j) => Math.abs(j.dth) > tol.th || Math.abs(j.dk) > tol.k);
    lines.push(`  #   ${d.id}: 경계 ${R.joints.length} 개 중 ` +
      `${opts.blend ? '평균으로 강제한 자리' : '위반'} ${bad.length} 개`);
    for (const j of bad) {
      lines.push(`  #     node ${base[d.id] + j.node}: ` +
        `dtheta ${(j.dth * 180 / Math.PI).toFixed(2)} deg, dkappa ${j.dk.toFixed(4)} 1/m`);
    }
  }
  lines.push('');

  download('lane_graph.yaml', lines.join('\n'));
  toast('lane_graph.yaml 저장');
}

function exportBoundary() {
  const spacing = parseFloat(document.getElementById('bndSpacing').value) || 0.1;
  const outer = S.layers.bnd_outer;
  const inner = S.layers.bnd_inner;

  if (outer.pts.length < 3 || inner.pts.length < 3) {
    toast('boundary 두 개가 각각 3점 이상이어야 한다');
    return;
  }

  const ring = {};
  for (const [key, L] of [['outer', outer], ['inner', inner]]) {
    ring[key] = resample(buildSegments(L.pts, L.closed), spacing, L.closed);
  }

  const arr = (vals) => {
    const rows = [];
    for (let i = 0; i < vals.length; i += 8) {
      rows.push('      ' + vals.slice(i, i + 8).map(fmt).join(', ') + ',');
    }
    return rows.join('\n');
  };

  const lines = [];
  lines.push('# KAU 실기 트랙 경계 — kau_global_path/web/lane_editor.html 로 생성');
  lines.push('#');
  lines.push('# 좌표는 map 프레임 미터다. amet_2026_track.yaml 은 Gazebo world');
  lines.push('# 프레임이라 실기에서는 못 쓴다 (kau_global_path/README 2.2).');
  lines.push('#');
  lines.push('# 주행 가능 고리 = outer 안쪽 AND inner 바깥쪽.');
  lines.push('# 이 파일에는 경계 폴리곤만 들어간다. 콘·장애물 위치는 담지 않는다.');
  lines.push(`# 두 폴리곤 모두 ${spacing.toFixed(2)} m 간격으로 곡선에서 다시 딴 점열이며 닫혀 있다.`);
  lines.push('#');
  lines.push('# 이 파일은 lane_graph 와 독립이다. Object Detection 은 lane_graph 를');
  lines.push('# 몰라도 되고, 이 패키지는 ROI 판정 로직을 몰라도 된다.');
  lines.push('');
  lines.push('laser_scan_clusterer:');
  lines.push('  ros__parameters:');
  lines.push(`    track_outer_x: [\n${arr(ring.outer.map((p) => p[0]))}\n    ]`);
  lines.push(`    track_outer_y: [\n${arr(ring.outer.map((p) => p[1]))}\n    ]`);
  lines.push(`    track_inner_x: [\n${arr(ring.inner.map((p) => p[0]))}\n    ]`);
  lines.push(`    track_inner_y: [\n${arr(ring.inner.map((p) => p[1]))}\n    ]`);
  lines.push('');

  download('kau_v3_track.yaml', lines.join('\n'));
  toast(`track_boundary 저장 (outer ${ring.outer.length} · inner ${ring.inner.length} 점)`);
}

// 우리가 쓴 lane_graph.yaml 만 다시 읽는다. 일반 YAML 파서가 아니다.
//
// 주의: 2026-08-21 에 id 대역이 바뀌었다. 예전에는 lane_inner 가 0 번대였고 지금은
// center 가 0 번대다. 그 전에 내보낸 파일을 열면 lane_inner 에 있던 점열이
// center 로 들어온다 — 우리 경우엔 거기 있던 게 주행면 중심선이라 그게 맞다.
// 진짜 차로 두 개짜리 옛 파일을 열 일이 생기면 id 를 손으로 100 씩 밀어야 한다.
function importLaneGraph(text) {
  const nodes = new Map();
  const re = /-\s*\{\s*id:\s*(\d+)\s*,\s*x:\s*([-\d.eE+]+)\s*,\s*y:\s*([-\d.eE+]+)/g;
  let m;
  while ((m = re.exec(text)) !== null) {
    nodes.set(parseInt(m[1], 10), [parseFloat(m[2]), parseFloat(m[3])]);
  }
  if (!nodes.size) { toast('노드를 못 찾았다'); return; }

  pushUndo();
  const base = LAYER_BASE;

  for (const d of LAYER_DEFS) {
    const pts = [];
    for (let i = 0; ; i++) {
      const p = nodes.get(base[d.id] + i);
      if (!p) break;
      pts.push(p);
    }
    const L = S.layers[d.id];
    nodeReset(L, pts);

    const cm = text.match(new RegExp(`^\\s*${d.id}:\\s*(true|false)`, 'm'));
    if (cm) L.closed = cm[1] === 'true';

    // segments 블록이 있으면 세그먼트 구분을 되살린다. 없으면 경계 없음 =
    // 예전 파일이고, 예전과 똑같이 전체 보간으로 열린다.
    const sm = text.match(new RegExp(
      `^\\s*${d.id}:\\s*\\{\\s*breaks:\\s*\\[([^\\]]*)\\]\\s*,\\s*links:\\s*\\[([^\\]]*)\\]`, 'm'));
    if (sm) {
      const nums = (t) => t.split(',').map((v) => parseInt(v, 10)).filter((v) => !isNaN(v));
      for (const id of nums(sm[1])) { const i = id - base[d.id]; if (i >= 0 && i < pts.length) L.brk[i] = true; }
      for (const id of nums(sm[2])) { const i = id - base[d.id]; if (i >= 0 && i < pts.length) L.link[i] = true; }
    }
  }

  renderLayers();
  draw();
  autosave();
  toast(`불러왔다: 노드 ${nodes.size} 개`);
}

// ---------------------------------------------------------------- 참조 이미지

function overlayInputs() {
  const O = S.overlay;
  document.getElementById('imgCx').value = O.cx.toFixed(3);
  document.getElementById('imgCy').value = O.cy.toFixed(3);
  document.getElementById('imgW').value = O.wm.toFixed(3);
  document.getElementById('imgRot').value = (O.rot * 180 / Math.PI).toFixed(2);
  document.getElementById('imgAlpha').value = O.alpha.toFixed(2);
  document.getElementById('chkImg').checked = O.on;
  document.getElementById('imgName').textContent = O.img ? `${O.name} (${O.img.width}x${O.img.height})` : '이미지 없음';
}

function overlayApply() {
  const O = S.overlay;
  O.cx = parseFloat(document.getElementById('imgCx').value) || 0;
  O.cy = parseFloat(document.getElementById('imgCy').value) || 0;
  O.wm = Math.max(0.05, parseFloat(document.getElementById('imgW').value) || 1);
  O.rot = (parseFloat(document.getElementById('imgRot').value) || 0) * Math.PI / 180;
  O.alpha = Math.min(1, Math.max(0, parseFloat(document.getElementById('imgAlpha').value)));
  if (isNaN(O.alpha)) O.alpha = 0.6;
  overlaySave();
  draw();
}

// 벽 경계상자에 맞춰 놓는다. 도면이든 위에서 찍은 사진이든 보통 방 전체가
// 찍혀 있으니 여기서 시작하면 미세조정만 남는다. 가로/세로 비율이 다르면
// 이미지가 다 보이도록 큰 쪽에 맞춘다 (잘리는 것보다 넘치는 게 맞추기 쉽다).
function overlayFitMap() {
  const O = S.overlay;
  if (!S.map) return;

  const b = S.map.wall || {
    x0: S.map.ox, y0: S.map.oy,
    x1: S.map.ox + S.map.w * S.map.res, y1: S.map.oy + S.map.h * S.map.res,
  };

  O.cx = (b.x0 + b.x1) / 2;
  O.cy = (b.y0 + b.y1) / 2;
  O.rot = 0;

  const bw = b.x1 - b.x0, bh = b.y1 - b.y0;
  const ar = O.img ? O.img.width / O.img.height : bw / bh;
  O.wm = (ar >= bw / bh) ? bw : bh * ar;

  overlayInputs(); overlaySave(); draw();
}

// 위치·크기는 항상 저장한다. 이미지 자체는 작을 때만 같이 넣는다 —
// localStorage 는 5 MB 안팎이라 사진 한 장으로 꽉 차면 lane_graph 자동저장이
// 같이 죽는다. 용량을 넘으면 조용히 위치만 남긴다.
const OVERLAY_MAX_BYTES = 1500000;

function overlaySave() {
  const O = S.overlay;
  const geo = { name: O.name, cx: O.cx, cy: O.cy, wm: O.wm, rot: O.rot, alpha: O.alpha, on: O.on };
  try {
    localStorage.setItem('kau_overlay', JSON.stringify(geo));
    if (O.dataUrl && O.dataUrl.length < OVERLAY_MAX_BYTES) {
      localStorage.setItem('kau_overlay_img', O.dataUrl);
    }
  } catch (e) {
    try { localStorage.removeItem('kau_overlay_img'); } catch (e2) { /* 무시 */ }
  }
}

function setOverlayImage(img, name, dataUrl, keepPlacement) {
  const O = S.overlay;
  O.img = img; O.name = name; O.dataUrl = dataUrl || null;
  if (!keepPlacement) overlayFitMap();
  overlayInputs(); overlaySave(); draw();
}

function loadOverlayFile(file) {
  const fr = new FileReader();
  fr.onload = () => {
    const img = new Image();
    img.onload = () => {
      setOverlayImage(img, file.name, fr.result, false);
      toast(`참조 이미지 ${file.name} — 맞춤 모드로 벽에 맞춘다`);
    };
    img.onerror = () => toast('이미지를 못 읽었다');
    img.src = fr.result;
  };
  fr.readAsDataURL(file);
}

function loadOverlayStored() {
  let geo = null;
  try { geo = JSON.parse(localStorage.getItem('kau_overlay') || 'null'); } catch (e) { /* 무시 */ }
  if (geo) Object.assign(S.overlay, geo);

  let url = null;
  try { url = localStorage.getItem('kau_overlay_img'); } catch (e) { /* 무시 */ }
  if (!url) { overlayInputs(); return; }

  const img = new Image();
  img.onload = () => { S.overlay.img = img; S.overlay.dataUrl = url; overlayInputs(); draw(); };
  img.src = url;
}

// ---------------------------------------------------------------- CAD 경로 생성

// CAD 중심선은 매끈한 곡선이 아니다. **곧은 구간과 뾰족한 꼭짓점으로 된 다각형**에
// 완만한 원호 몇 개가 섞인 모양이고, 꼭짓점 꺾임각이 6 ~ 92 도다. 차는 그걸
// 그대로 못 돈다 (최소회전반경 0.494 m). 그래서 꼭짓점마다 반지름 Rf 의 원호로
// 둥글린다 — 토목에서 도로 평면선형 잡는 것과 같은 방식이다.
//
// 접선길이 T = Rf * |tan(Δ/2)| 를 양쪽 직선에서 떼어 쓴다. 이웃한 두 꼭짓점의
// T 합이 사이 직선보다 길면 둘 다 같은 비율로 줄인다 — 한쪽만 줄이면 그 꼭짓점만
// 유난히 뾰족해진다. 다 줄이고도 모자란 자리는 반지름이 목표에 못 미치니 그대로
// 보고한다 (숨기면 발행하고 나서 못 도는 코너를 만나게 된다).
function cadCorner(v, prevOut, nextIn, Rf, gentleDeg, Rcap) {
  const u = [v[0] - prevOut[0], v[1] - prevOut[1]];
  const w = [nextIn[0] - v[0], nextIn[1] - v[1]];
  const nu = Math.hypot(u[0], u[1]);
  const nw = Math.hypot(w[0], w[1]);
  if (nu < 1e-9 || nw < 1e-9) return null;

  u[0] /= nu; u[1] /= nu;
  w[0] /= nw; w[1] /= nw;

  const d = Math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]);
  const tanHalf = Math.abs(Math.tan(d / 2));
  if (tanHalf < 1e-6) return null;         // 사실상 직선

  // 완만한 꼭짓점은 반지름을 키운다. CAD 의 완만한 곡선(R 1.5 ~ 3.8 m)이
  // 여기서는 작은 각의 꼭짓점 몇 개로 나타나는데, 전부 Rf 로 둥글리면 원래
  // 완만하던 자리가 |κ| = 1/Rf 까지 올라간다. 접선길이는 T = R tan(Δ/2) 라
  // 각이 작으면 R 을 키워도 거의 안 먹는다.
  const tgt = Math.abs(d) * 180 / Math.PI <= gentleDeg ? Math.max(Rf, Rcap) : Rf;
  return { v, u, w, d, tanHalf, T: tgt * tanHalf, R: tgt, floor: Rf * tanHalf,
           runIn: nu, runOut: nw };
}

function arcThrough(A, B, R, left) {
  // A 에서 B 로 반지름 R 로 휘어 가는 원호의 중심과 중점.
  //
  // 곡률 중심은 **진행 방향의 안쪽**에 있다. 좌회전이면 현 A->B 의 왼쪽,
  // 우회전이면 오른쪽이다. 부호를 뒤집으면 반지름은 맞는데 코너를 안쪽으로
  // 자르는 대신 바깥으로 부풀어서, 접선 방향이 통째로 뒤집힌 원호가 나온다.
  const ux = B[0] - A[0], uy = B[1] - A[1];
  const c = Math.hypot(ux, uy);
  if (c < 1e-12 || c > 2 * R) return null;
  const mx = (A[0] + B[0]) / 2, my = (A[1] + B[1]) / 2;
  const h = Math.sqrt(Math.max(0, R * R - c * c / 4));
  const nx = -uy / c, ny = ux / c;              // 현의 왼쪽 법선
  const sgn = left ? 1 : -1;
  const cen = [mx + sgn * h * nx, my + sgn * h * ny];
  const dx = mx - cen[0], dy = my - cen[1];
  const dn = Math.hypot(dx, dy) || 1;
  return { cen, mid: [cen[0] + R * dx / dn, cen[1] + R * dy / dn] };
}

// 골격 -> 노드 + 세그먼트 플래그. 편집기의 세그먼트 모델과 그대로 맞는다.
//   직선 구간 -> 노드 2 개짜리 line
//   원호(CAD 본래 것 + 꼭짓점 필렛) -> 노드 3 개짜리 arc
function buildCadPath(sk, Rf, opts = {}) {
  const margin = opts.margin ?? 0.02;
  const n = sk.length;
  const P = (q) => trackToMap(q);
  const entry = (it) => P(it.kind === 'arc' ? it.p0 : it.p);
  const exitp = (it) => P(it.kind === 'arc' ? it.p1 : it.p);

  const gentle = opts.gentleDeg ?? 20;
  const Rcap = opts.Rcap ?? 1.5;

  const info = sk.map((it, i) => it.kind === 'corner'
    ? cadCorner(P(it.p), exitp(sk[(i - 1 + n) % n]), entry(sk[(i + 1) % n]), Rf, gentle, Rcap)
    : null);

  // 접선길이 나눠 쓰기. 이웃한 두 꼭짓점의 T 합이 사이 직선보다 길면 줄인다.
  // **Rf 위로 키워 놓은 쪽부터 먼저 깎는다** — 비례로 줄이면 급한 코너가
  // 완만한 코너 때문에 같이 작아져서, 정작 못 도는 자리가 생긴다.
  for (let pass = 0; pass < 60; pass++) {
    let changed = false;
    for (let i = 0; i < n; i++) {
      const a = info[i];
      if (!a) continue;
      const b = info[(i + 1) % n];
      const avail = a.runOut - margin;
      let need = a.T + (b ? b.T : 0);
      if (need <= avail || need <= 1e-9) continue;

      let over = need - avail;
      for (const x of [a, b].filter(Boolean).sort((p, q) => (q.T - q.floor) - (p.T - p.floor))) {
        const slack = Math.max(0, x.T - x.floor);
        const cut = Math.min(over, slack);
        x.T -= cut; over -= cut;
      }
      if (over > 1e-12) {
        const tot = a.T + (b ? b.T : 0);
        const f = tot > 1e-12 ? Math.max(0, avail) / tot : 0;
        a.T *= f;
        if (b) b.T *= f;
      }
      a.R = a.T / a.tanHalf;
      if (b) b.R = b.T / b.tanHalf;
      changed = true;
    }
    if (!changed) break;
  }

  // 원호 조각들. 사이는 직선이다.
  const els = [];
  let kmax = 0, rmin = Infinity, clamped = 0;

  for (let i = 0; i < n; i++) {
    const it = sk[i];
    if (it.kind === 'arc') {
      els.push({ A: P(it.p0), M: P(it.pm), B: P(it.p1) });
      kmax = Math.max(kmax, 1 / it.R);
      rmin = Math.min(rmin, it.R);
      continue;
    }
    const a = info[i];
    if (!a) continue;

    const A = [a.v[0] - a.u[0] * a.T, a.v[1] - a.u[1] * a.T];
    const B = [a.v[0] + a.w[0] * a.T, a.v[1] + a.w[1] * a.T];
    const g = arcThrough(A, B, a.R, a.d > 0);
    if (!g) continue;

    els.push({ A, M: g.mid, B });
    kmax = Math.max(kmax, 1 / a.R);
    rmin = Math.min(rmin, a.R);
    if (a.R < Rf - 1e-4) clamped++;
  }

  // 노드로 편다. 원호는 (A, M, B) 세 개, 사이 직선은 B -> 다음 A 다.
  //
  // 전이를 켜면 직선 양 끝에서 Lt 씩 떼어 link 세그먼트로 만든다. 직선(κ=0)과
  // 원호(κ=1/R)는 맞닿는 한 C² 가 안 되므로, 그 사이에서 곡률을 흡수할 구간이
  // 있어야 조향각이 계단으로 튀지 않는다 (4.4.1).
  const Lt = opts.transition || 0;
  const pts = [], brk = [], link = [];
  const push = (p, b, l) => {
    if (pts.length && dist(pts[pts.length - 1], p) < 0.002) { brk[brk.length - 1] = brk[brk.length - 1] || b; return; }
    pts.push(p); brk.push(b); link.push(!!l);
  };

  const m = els.length;
  let short = 0, links = 0;

  for (let i = 0; i < m; i++) {
    const e = els[i];
    push(e.A, true, false);
    push(e.M, false, false);
    push(e.B, true, false);

    if (!Lt) continue;

    const nx = els[(i + 1) % m].A;
    const Ls = dist(e.B, nx);
    if (Ls < 0.02) continue;

    const u = [(nx[0] - e.B[0]) / Ls, (nx[1] - e.B[1]) / Ls];

    if (Ls >= 2 * Lt + 0.05) {
      link[link.length - 1] = true;                       // B -> P1 = 전이
      push([e.B[0] + u[0] * Lt, e.B[1] + u[1] * Lt], true, false);   // P1 -> P2 = 직선
      push([nx[0] - u[0] * Lt, nx[1] - u[1] * Lt], true, true);      // P2 -> A' = 전이
      links += 2;
    } else {
      link[link.length - 1] = true;                       // 직선이 짧다. 통째로 전이
      links++;
      short++;
    }
  }

  if (pts.length > 2 && dist(pts[0], pts[pts.length - 1]) < 0.002) {
    pts.pop(); brk.pop(); link.pop();
  }

  return { pts, brk, link, kmax, rmin, clamped, arcs: m, links, shortStraights: short };
}

function trackInputs() {
  const T = S.track;
  document.getElementById('trkOx').value = T.ox.toFixed(3);
  document.getElementById('trkOy').value = T.oy.toFixed(3);
  document.getElementById('trkRot').value = (T.rot * 180 / Math.PI).toFixed(2);
  document.getElementById('trkAlpha').value = T.alpha.toFixed(2);
  document.getElementById('chkTrack').checked = T.on;
  document.getElementById('chkTrackRoad').checked = T.road;
  document.getElementById('trkInfo').textContent = T.data
    ? `${T.data.source} · ${T.data.world_size_m.join(' x ')} m`
    : 'config/amet2026_track.json 없음 — scripts/extract_sim_track.py 로 만든다';
}

function trackApply() {
  const T = S.track;
  T.ox = parseFloat(document.getElementById('trkOx').value) || 0;
  T.oy = parseFloat(document.getElementById('trkOy').value) || 0;
  T.rot = (parseFloat(document.getElementById('trkRot').value) || 0) * Math.PI / 180;
  const a = parseFloat(document.getElementById('trkAlpha').value);
  T.alpha = isNaN(a) ? 0.85 : Math.min(1, Math.max(0, a));
  trackSave();
  draw();
}

function trackSave() {
  const T = S.track;
  try {
    localStorage.setItem('kau_track', JSON.stringify(
      { ox: T.ox, oy: T.oy, rot: T.rot, alpha: T.alpha, on: T.on, road: T.road }));
  } catch (e) { /* 무시 */ }
}

function loadTrackFromServer() {
  return fetch(TRACK_JSON).then((r) => {
    if (!r.ok) throw new Error('no track');
    return r.json();
  }).then((doc) => {
    S.track.data = doc;

    // 주행면 폭은 CAD 에서 잰 값을 그대로 쓴다. 예전 차로폭 기본값 0.40 을
    // 남겨 두면 내보낸 파일이 실측과 어긋난다.
    const rc = doc.centerlines && doc.centerlines.road_center;
    if (rc && rc.corridor_half_width_m) {
      document.getElementById('roadWidth').value = (rc.corridor_half_width_m * 2).toFixed(3);
    }
    // 파일이 들고 있는 기본 정렬값. 사용자가 손댄 적 있으면 그쪽이 이긴다.
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem('kau_track') || 'null'); } catch (e) { /* 무시 */ }
    if (saved) Object.assign(S.track, saved);
    else if (doc.sim_to_map) {
      S.track.ox = doc.sim_to_map.ox;
      S.track.oy = doc.sim_to_map.oy;
      S.track.rot = (doc.sim_to_map.rot_deg || 0) * Math.PI / 180;
    }
    trackInputs();
  }).catch(() => { trackInputs(); });
}

// web/ 에 overlay.png 를 두면 자동으로 뜬다. scp 로 올려 놓고 새로고침만 하면
// 되니 매번 끌어다 놓을 필요가 없다.
function loadOverlayFromServer() {
  return fetch('overlay.png', { method: 'HEAD' }).then((r) => {
    if (!r.ok || S.overlay.img) return;
    const img = new Image();
    img.onload = () => { setOverlayImage(img, 'overlay.png', null, false); };
    img.src = 'overlay.png';
  }).catch(() => {});
}

// ---------------------------------------------------------------- 버튼

document.getElementById('btnReload').onclick = () => {
  loadMapFromServer().then(() => loadTrajectories()).then(draw)
    .catch((e) => toast(e.message));
};

document.getElementById('btnFit').onclick = fitView;

function setCurvePreview(on) {
  S.show.curve = on;
  document.getElementById('chkCurve').checked = on;
  draw();
}
document.getElementById('chkCtrl').checked = S.show.ctrl;
document.getElementById('chkCtrl').onchange = (e) => { S.show.ctrl = e.target.checked; draw(); };
document.getElementById('chkCurve').checked = S.show.curve;
document.getElementById('chkCurve').onchange = (e) => setCurvePreview(e.target.checked);

function applyDisp() {
  DISP.on = document.getElementById('chkDisp').checked;
  DISP.ox = parseFloat(document.getElementById('dispOx').value) || 0;
  DISP.oy = parseFloat(document.getElementById('dispOy').value) || 0;
  draw();
}
// 브라우저가 새로고침 때 체크박스 상태를 복원해 버리면 기본값이 뒤집힌다.
// 시작할 때 한 번 강제로 맞추고 적용한다.
document.getElementById('build').textContent = BUILD;
document.getElementById('chkDisp').checked = DISP.on;
document.getElementById('dispOx').value = DISP.ox;
document.getElementById('dispOy').value = DISP.oy;
applyDisp();
document.getElementById('chkDisp').onchange = applyDisp;
document.getElementById('dispOx').onchange = applyDisp;
document.getElementById('dispOy').onchange = applyDisp;

document.getElementById('chkClosed').onchange = (e) => {
  pushUndo();
  S.layers[S.active].closed = e.target.checked;
  renderLayers(); draw(); autosave();
};

document.getElementById('lapPick').onchange = (e) => {
  const def = LAYER_DEFS.find((d) => d.id === S.active);
  if (!def || def.kind !== 'lane') return;
  const v = e.target.value;
  S.lapSel[def.lane] = v === 'avg' ? v : Number(v);
  draw();
};

function seedGuard() {
  const def = LAYER_DEFS.find((d) => d.id === S.active);
  if (def.kind !== 'lane') { toast('주행 기록이 있는 차로 레이어에서만 쓴다 (지금 경로는 CAD 에서 만든다)'); return null; }
  if (!S.laps[def.lane].length) {
    toast(`data/${def.lane}_1.csv 가 없다 — record_trajectory.py 로 먼저 주행`);
    return null;
  }
  return def;
}

document.getElementById('btnSeedAdapt').onclick = () => {
  const def = seedGuard();
  if (!def) return;

  pushUndo();
  const tol = parseFloat(document.getElementById('seedTol').value) || 0.02;
  const minSpacing = parseFloat(document.getElementById('seedMin').value) || 0.15;
  const L = S.layers[S.active];
  const r = seedAdaptive(seedSource(def.lane), tol, L.closed, { minSpacing });
  nodeReset(L, r.pts);
  renderLayers(); draw(); autosave();
  toast(r.blocked
    ? `노드 ${L.pts.length} 개 — 최소 간격 ${(minSpacing * 100).toFixed(0)} cm 에 걸려 ` +
      `오차 ${(r.dev * 100).toFixed(1)} cm 에서 멈췄다 (목표 ${(tol * 100).toFixed(1)} cm)`
    : `노드 ${L.pts.length} 개 — 궤적 오차 최대 ${(r.dev * 100).toFixed(1)} cm`);
};

document.getElementById('btnSeed').onclick = () => {
  const def = LAYER_DEFS.find((d) => d.id === S.active);
  if (def.kind !== 'lane') { toast('주행 기록이 있는 차로 레이어에서만 쓴다 (지금 경로는 CAD 에서 만든다)'); return; }
  if (!S.laps[def.lane].length) {
    toast(`data/${def.lane}_1.csv 가 없다 — record_trajectory.py 로 먼저 주행`);
    return;
  }

  pushUndo();
  const sp = parseFloat(document.getElementById('seedSpacing').value) || 0.5;
  nodeReset(S.layers[S.active], seedFromTrajectory(seedSource(def.lane), sp));
  renderLayers(); draw(); autosave();

  const laps = S.laps[def.lane];
  const from = S.lapSel[def.lane] === 'avg' ? `${laps.length} 바퀴 평균`
    : laps[S.lapSel[def.lane]].name;
  toast(`${from} 에서 ${S.layers[S.active].pts.length} 개 노드 — 이제 손으로 다듬는다`);
};

document.getElementById('btnClear').onclick = () => {
  pushUndo();
  nodeReset(S.layers[S.active], []);
  renderLayers(); draw(); autosave();
};

document.getElementById('btnReverse').onclick = () => {
  pushUndo();
  reverseLayer(S.layers[S.active]);
  renderLayers(); draw(); autosave();
};

document.getElementById('btnBnd').onclick = () => {
  const w = parseFloat(document.getElementById('laneWidth').value) || 0.4;
  const outer = S.layers.lane_outer, inner = S.layers.lane_inner;

  if (outer.pts.length < 3 || inner.pts.length < 3) {
    toast('두 중심선을 먼저 그려야 한다');
    return;
  }

  pushUndo();
  nodeReset(S.layers.bnd_outer, offsetPolyline(outer.pts, outer.closed, w / 2, true));
  S.layers.bnd_outer.closed = outer.closed;
  nodeReset(S.layers.bnd_inner, offsetPolyline(inner.pts, inner.closed, w / 2, false));
  S.layers.bnd_inner.closed = inner.closed;

  renderLayers(); draw(); autosave();
  toast('boundary 생성 — 벽/테이프 위치와 대조해서 손으로 다듬을 것');
};

// 차량 제원 -> 곡률 한계. 세 값 중 하나라도 바뀌면 다시 계산해서 kappaLimit 에
// 써 넣는다. kappaLimit 자체도 여전히 손으로 고칠 수 있게 남겨 둔다 — 디버깅할 때
// 한계만 잠깐 풀어 보고 싶은 경우가 있다.
function applyVeh(recompute) {
  VEH.wheelbase = parseFloat(document.getElementById('vehL').value) || 0.18;
  VEH.maxSteerDeg = parseFloat(document.getElementById('vehSteer').value) || 20;
  VEH.margin = parseFloat(document.getElementById('vehMargin').value) || 0.9;

  const kmax = kappaMax();
  const lim = kmax * VEH.margin;
  if (recompute) document.getElementById('kappaLimit').value = lim.toFixed(4);

  const cur = curvLimit();
  document.getElementById('vehInfo').innerHTML =
    `R_min = L/tan δ = ${(1 / kmax).toFixed(4)} m · ` +
    `κ_max = ${kmax.toFixed(4)} 1/m<br>` +
    `한계 = κ_max × ${VEH.margin.toFixed(2)} = <b>${lim.toFixed(4)} 1/m</b> ` +
    `(R ${(1 / lim).toFixed(3)} m · δ ${steerOfKappa(lim).toFixed(1)}°)` +
    (Math.abs(cur - lim) > 1e-6
      ? `<br><span class="bad">지금 쓰는 한계는 ${cur.toFixed(4)} — 손으로 덮어썼다</span>` : '');

  runCheck();
  draw();
}

document.getElementById('kappaLimit').oninput = () => applyVeh(false);
for (const id of ['vehL', 'vehSteer', 'vehMargin']) {
  document.getElementById(id).oninput = () => applyVeh(true);
}
document.getElementById('btnVehReset').onclick = () => {
  document.getElementById('vehL').value = '0.18';
  document.getElementById('vehSteer').value = '20';
  document.getElementById('vehMargin').value = '0.90';
  applyVeh(true);
};
document.getElementById('chkBlend').onchange = () => { runCheck(); draw(); autosave(); };
document.getElementById('c2Theta').onchange = () => { runCheck(); draw(); };
document.getElementById('c2Kappa').onchange = () => { runCheck(); draw(); };

document.getElementById('btnBrkAll').onclick = () => {
  const L = S.layers[S.active];
  if (!L.pts.length) return;
  pushUndo();
  L.brk = L.pts.map(() => true);
  renderLayers(); draw(); autosave();
  toast(`노드 ${L.pts.length} 개가 전부 경계 — 전 구간이 노드 2 개짜리 직선이 된다`);
};

document.getElementById('btnBrkNone').onclick = () => {
  const L = S.layers[S.active];
  pushUndo();
  L.brk = L.pts.map(() => false);
  L.link = L.pts.map(() => false);
  renderLayers(); draw(); autosave();
  toast('경계 해제 — 전체를 한 덩어리로 보간한다 (예전 동작)');
};
function setImgFit(on) {
  S.overlay.fit = on;
  document.getElementById('chkImgFit').checked = on;
  cv.style.cursor = on ? 'move' : 'crosshair';
  if (on) toast('맞춤 모드 — 드래그 이동 · 휠 확대 · [ ] 회전 (노드 편집은 잠깐 멈춤)');
  draw();
}

document.getElementById('chkImg').onchange = (e) => {
  S.overlay.on = e.target.checked; overlaySave(); draw();
};
document.getElementById('chkImgFit').onchange = (e) => setImgFit(e.target.checked);
for (const id of ['imgCx', 'imgCy', 'imgW', 'imgRot', 'imgAlpha']) {
  document.getElementById(id).oninput = overlayApply;
}
document.getElementById('btnImgFitMap').onclick = () => {
  overlayFitMap();
  toast('지도 전체를 덮게 놓았다 — 이제 벽 모서리에 맞춘다');
};
document.getElementById('btnImgPick').onclick = () => document.getElementById('fileImg').click();
document.getElementById('fileImg').onchange = (e) => {
  const f = e.target.files[0];
  if (f) loadOverlayFile(f);
  e.target.value = '';
};
document.getElementById('btnImgClear').onclick = () => {
  S.overlay.img = null; S.overlay.dataUrl = null; S.overlay.name = '';
  try { localStorage.removeItem('kau_overlay_img'); } catch (err) { /* 무시 */ }
  overlayInputs(); overlaySave(); draw();
};

document.getElementById('chkTrack').onchange = (e) => {
  S.track.on = e.target.checked; trackSave(); draw();
};
document.getElementById('chkTrackRoad').onchange = (e) => {
  S.track.road = e.target.checked; trackSave(); draw();
};
for (const id of ['trkOx', 'trkOy', 'trkRot', 'trkAlpha']) {
  document.getElementById(id).oninput = trackApply;
}
document.getElementById('btnCadPath').onclick = () => {
  const T = S.track;
  if (!T.data || !T.data.centerlines) { toast('config/amet2026_track.json 이 없다'); return; }

  const key = document.getElementById('cadSrc').value;
  const c = T.data.centerlines[key];
  if (!c || !c.skeleton) { toast(`${key} 골격이 없다 — extract_sim_track.py 를 다시 돌릴 것`); return; }

  // 기준선과 레이어를 사람이 맞추게 두면 반드시 어긋난다. 기준선이 정하게 한다.
  const target = { road_center: 'center', lane_outer: 'lane_outer', lane_inner: 'lane_inner' }[key];
  if (target && S.active !== target) S.active = target;

  const Rf = parseFloat(document.getElementById('cadR').value) || 0.60;
  const Lt = document.getElementById('chkCadTrans').checked
    ? (parseFloat(document.getElementById('cadTrans').value) || 0) : 0;
  const Rcap = parseFloat(document.getElementById('cadRcap').value) || Rf;
  const limit = curvLimit();
  const r = buildCadPath(c.skeleton, Rf, { transition: Lt, Rcap });

  pushUndo();
  const L = S.layers[S.active];
  nodeReset(L, r.pts);
  L.brk = r.brk;
  L.link = r.link;
  L.closed = true;
  renderLayers(); draw(); autosave();

  const over = r.kmax > limit;
  toast(`${c.label}: 노드 ${r.pts.length} · 원호 ${r.arcs} · 전이 ${r.links} · ` +
    `R_min ${r.rmin.toFixed(3)} m (|k| ${r.kmax.toFixed(3)})` +
    (r.clamped ? ` · 직선이 짧아 R 못 채운 꼭짓점 ${r.clamped}` : '') +
    (r.shortStraights ? ` · 직선이 짧아 통째로 전이가 된 곳 ${r.shortStraights}` : '') +
    (over ? ` · 곡률 한계 ${limit} 초과!` : ''));
};

document.getElementById('cadSrc').onchange = draw;

document.getElementById('btnTrackReset').onclick = () => {
  const d = (S.track.data && S.track.data.sim_to_map) || { ox: 3.68, oy: 1.39, rot_deg: 0 };
  S.track.ox = d.ox; S.track.oy = d.oy; S.track.rot = (d.rot_deg || 0) * Math.PI / 180;
  trackInputs(); trackSave(); draw();
  toast('CAD 정렬을 파일 기본값으로 되돌렸다');
};

document.getElementById('btnExportGraph').onclick = exportLaneGraph;
document.getElementById('btnExportBnd').onclick = exportBoundary;

document.getElementById('btnImport').onclick = () =>
  document.getElementById('fileImport').click();

document.getElementById('fileImport').onchange = (e) => {
  const f = e.target.files[0];
  if (!f) return;
  f.text().then(importLaneGraph);
  e.target.value = '';
};

// ---------------------------------------------------------------- 드래그 앤 드롭

window.addEventListener('dragover', (e) => e.preventDefault());

window.addEventListener('drop', async (e) => {
  e.preventDefault();

  for (const f of e.dataTransfer.files) {
    const name = f.name.toLowerCase();

    if (/\.(png|jpe?g|webp|gif|bmp)$/.test(name)) {
      loadOverlayFile(f);
      continue;
    }

    if (name.endsWith('.pgm')) {
      pendingPgm = { name: f.name.replace(/\.pgm$/i, ''), buf: await f.arrayBuffer() };
    } else if (name.endsWith('.yaml') || name.endsWith('.yml')) {
      const text = await f.text();
      if (/lane_graph\s*:/.test(text)) { importLaneGraph(text); continue; }
      pendingYaml = text;
    } else if (name.endsWith('.csv')) {
      // 미완주 / 진행 중 파일은 평균 대상이 아니다. `inner_partial_1.csv` 가
      // `_1.csv` 로 끝나므로 바퀴 파일 판정보다 **먼저** 걸러야 한다.
      if (/_(partial_\d+|recording|raw)\.csv$/.test(name)) {
        toast(`${f.name} 은 평균 대상이 아니다 (미완주 / 진행 중)`);
        continue;
      }

      const lane = name.includes('outer') ? 'outer' : 'inner';
      const pts = parseCSV(await f.text());

      if (/_(\d+)\.csv$/.test(name)) {
        addLap(lane, f.name, pts);
      } else {
        setLaps(lane, splitLaps(pts).map((l, i) => ({ name: `${f.name} #${i + 1}`, pts: l })));
      }

      updateTrajInfo();
      toast(`${lane}: ${S.laps[lane].length} 바퀴`);
    }
  }

  if (pendingPgm && pendingYaml) {
    try { setMap(pendingPgm.name, pendingPgm.buf, pendingYaml); toast('지도 로드'); }
    catch (err) { toast(err.message); }
    pendingPgm = pendingYaml = null;
  } else if (pendingPgm || pendingYaml) {
    toast('pgm 과 yaml 을 같이 놓아야 한다');
  }

  draw();
});

// ---------------------------------------------------------------- 기동

window.addEventListener('resize', resize);

applyVeh(true);
renderLayers();
loadAutosave();
loadOverlayStored();
resize();

loadMapFromServer()
  .then(() => loadTrajectories())
  .then(() => loadTrackFromServer())
  .then(() => loadOverlayFromServer())
  .then(draw)
  .catch(() => {
    toast('지도 자동 로드 실패 — 파일을 끌어다 놓거나 http.server 로 열 것');
    draw();
  });
