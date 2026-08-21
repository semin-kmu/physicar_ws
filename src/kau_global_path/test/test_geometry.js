// lane_editor.js 의 곡선 기하 검증. ROS 도 브라우저도 필요 없다.
//
//   node src/kau_global_path/test/test_geometry.js
//
// 편집기는 DOM 스크립트라 import 가 안 된다. vm 컨텍스트에 최소 stub 을 깔고
// 통째로 실행한 뒤 전역에 남은 함수를 꺼내 쓴다.

'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const stubEl = () => new Proxy({}, {
  get(t, k) {
    if (k === 'value') return t.__v ?? '0.4';
    if (k === 'style') return {};
    if (k === 'getBoundingClientRect') return () => ({ width: 800, height: 600, left: 0, top: 0 });
    if (k === 'getContext') return () => new Proxy({}, { get: () => () => ({ data: [] }) });
    if (k === 'querySelector') return () => stubEl();
    if (k === 'querySelectorAll') return () => [];
    if (['appendChild', 'addEventListener', 'setAttribute', 'click'].includes(k)) return () => {};
    return t[k];
  },
  set(t, k, v) { if (k === 'value') t.__v = v; else t[k] = v; return true; },
});

const els = {};
const sandbox = {
  document: {
    getElementById: (id) => (els[id] || (els[id] = stubEl())),
    createElement: () => stubEl(),
    body: stubEl(),
  },
  window: { addEventListener() {}, devicePixelRatio: 1 },
  localStorage: { getItem: () => null, setItem() {} },
  fetch: () => Promise.reject(new Error('no server')),
  setTimeout, clearTimeout, console,
  URL: { createObjectURL: () => '', revokeObjectURL() {} },
  Blob: function () {},
  JSON, Math, Number, Array, String, Object, Promise, Uint8Array,
  Set, Map, RegExp, isFinite,
  Infinity, NaN, isNaN, parseFloat, parseInt,
};
sandbox.globalThis = sandbox;

vm.createContext(sandbox);
vm.runInContext(
  fs.readFileSync(path.join(__dirname, '..', 'web', 'lane_editor.js'), 'utf8'),
  sandbox);

const { buildSegments, segLength, segKappaMax, knotFrames, offsetPolyline, resample,
        splitLaps, alignLaps, resampleClosedFrom, seedFromTrajectory,
        seedAdaptive, distToPolyline, deCasteljau,
        buildLayer, splitSegments, arcFrames, reverseLayer, nodeInsert, nodeRemove,
        hermiteToBezier } = sandbox;

let failed = 0;

function check(name, ok, detail) {
  console.log(`${ok ? '  통과' : '  실패'}  ${name}${detail ? '   ' + detail : ''}`);
  if (!ok) failed++;
}

function circle(R, n) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = 2 * Math.PI * i / n;
    pts.push([R * Math.cos(a), R * Math.sin(a)]);
  }
  return pts;
}

console.log('\n[1] 원호 재현 — 노드 곡률은 외접원에서 뽑으므로 정확해야 한다');
for (const [R, n] of [[1.5, 8], [1.5, 16], [0.8, 12], [3.0, 20]]) {
  const pts = circle(R, n);
  const kn = Math.max(...knotFrames(pts, true).map((f) => Math.abs(f.k)));
  const segs = buildSegments(pts, true);
  const len = segs.reduce((s, c) => s + segLength(c), 0);
  const kseg = Math.max(...segs.map((c) => segKappaMax(c)));

  const lenErr = Math.abs(len - 2 * Math.PI * R) / (2 * Math.PI * R);
  // 노드 곡률은 원호에서 정확. segment 내부는 quintic Hermite 고유의 부풀림이
  // 남는다 (fake_path.py 에 정확한 kappa 를 넣어도 같은 값이 나온다).
  // 항상 위로 부풀기 때문에 곡률 경고는 보수적으로 동작한다.
  check(`R=${R} n=${n} 노드 간격 ${(2 * Math.PI * R / n).toFixed(2)} m`,
    Math.abs(kn * R - 1) < 1e-9 && lenErr < 1e-3 && kseg >= 1 / R && kseg * R - 1 < 0.05,
    `길이오차 ${(lenErr * 100).toFixed(3)}%  노드 k ${(kn * R).toFixed(6)}/R  ` +
    `내부 최대 +${((kseg * R - 1) * 100).toFixed(1)}%`);
}

console.log('\n[2] 곡선이 찍은 노드를 정확히 지나는가 (Bezier 제어점이 아니다)');
{
  const pts = [[0, 0], [1, 0.4], [2.3, 0.1], [3, 1.5], [1.2, 2.2], [-0.4, 1.1]];
  for (const closed of [true, false]) {
    const segs = buildSegments(pts, closed);
    let worst = 0;
    segs.forEach((c, i) => {
      worst = Math.max(worst, Math.hypot(c[0][0] - pts[i][0], c[0][1] - pts[i][1]));
      const j = (i + 1) % pts.length;
      worst = Math.max(worst, Math.hypot(c[5][0] - pts[j][0], c[5][1] - pts[j][1]));
    });
    check(`통과 (closed=${closed})`, worst < 1e-12, `최대 오차 ${worst.toExponential(2)} m`);
  }
}

console.log('\n[3] segment 이음새 G2 — 시작-끝 wrap 포함 (README 6.5)');
{
  const pts = [[0, 0], [1, 0.4], [2.3, 0.1], [3, 1.5], [1.2, 2.2], [-0.4, 1.1]];
  const segs = buildSegments(pts, true);
  const hodo = (c) => {
    const n = c.length - 1;
    return c.slice(0, n).map((_, i) => [n * (c[i + 1][0] - c[i][0]), n * (c[i + 1][1] - c[i][1])]);
  };
  const dc = (c, u) => {
    const p = c.map((q) => [q[0], q[1]]);
    for (let r = 0; r < p.length - 1; r++) {
      for (let i = 0; i < p.length - 1 - r; i++) {
        p[i] = [(1 - u) * p[i][0] + u * p[i + 1][0], (1 - u) * p[i][1] + u * p[i + 1][1]];
      }
    }
    return p[0];
  };
  const state = (c, u) => {
    const d1 = hodo(c), d2 = hodo(d1);
    const v = dc(d1, u), a = dc(d2, u);
    const sp = Math.hypot(v[0], v[1]);
    return [Math.atan2(v[1], v[0]), (v[0] * a[1] - v[1] * a[0]) / (sp ** 3)];
  };

  let dth = 0, dk = 0;
  for (let i = 0; i < segs.length; i++) {
    const [t1, k1] = state(segs[i], 1);
    const [t2, k2] = state(segs[(i + 1) % segs.length], 0);
    let d = Math.abs(t1 - t2);
    d = Math.min(d, Math.abs(2 * Math.PI - d));
    dth = Math.max(dth, d);
    dk = Math.max(dk, Math.abs(k1 - k2));
  }
  check('heading / 곡률 연속', dth < 1e-9 && dk < 1e-7,
    `dtheta ${dth.toExponential(2)} rad  dkappa ${dk.toExponential(2)} 1/m`);
}

console.log('\n[4] boundary offset — 부호와 크기');
{
  const R = 2.0, d = 0.2;
  const pts = circle(R, 24);
  const rad = (P) => P.map((p) => Math.hypot(p[0], p[1]));
  const eo = Math.max(...rad(offsetPolyline(pts, true, d, true)).map((r) => Math.abs(r - (R + d))));
  const ei = Math.max(...rad(offsetPolyline(pts, true, d, false)).map((r) => Math.abs(r - (R - d))));
  check('CCW', eo < 1e-9 && ei < 1e-9, `outer ${eo.toExponential(2)}  inner ${ei.toExponential(2)}`);

  const cw = circle(R, 24).reverse();
  const eo2 = Math.max(...rad(offsetPolyline(cw, true, d, true)).map((r) => Math.abs(r - (R + d))));
  const ei2 = Math.max(...rad(offsetPolyline(cw, true, d, false)).map((r) => Math.abs(r - (R - d))));
  check('CW (진행방향 반대)', eo2 < 1e-9 && ei2 < 1e-9,
    `outer ${eo2.toExponential(2)}  inner ${ei2.toExponential(2)}`);
}

console.log('\n[5] 호길이 등간격 리샘플 — t 등분이 아니어야 한다');
{
  const R = 1.5;
  const rs = resample(buildSegments(circle(R, 12), true), 0.10, true);
  const gaps = [];
  for (let i = 1; i < rs.length; i++) gaps.push(Math.hypot(rs[i][0] - rs[i - 1][0], rs[i][1] - rs[i - 1][1]));
  const mn = Math.min(...gaps), mx = Math.max(...gaps);
  check('간격 균일', mx - mn < 2e-3 && rs.length === Math.ceil(2 * Math.PI * R / 0.10),
    `${rs.length} 점  간격 ${mn.toFixed(4)} ~ ${mx.toFixed(4)} m`);
}

console.log('\n[6] 개곡선 끝점 프레임 — 한쪽 차분으로 무너지지 않는가');
{
  const f = knotFrames([[0, 0], [1, 0], [2, 0], [3, 0]], false);
  check('직선', f.every((x) => Math.abs(x.th) < 1e-12 && Math.abs(x.k) < 1e-12),
    `th ${f.map((x) => x.th.toFixed(3)).join(' ')}`);

  const R = 2.0, g = knotFrames(circle(R, 24).slice(0, 6), false);
  check('원호 일부', g.every((x) => Math.abs(Math.abs(x.k) * R - 1) < 1e-9),
    `k*R ${g.map((x) => (Math.abs(x.k) * R).toFixed(4)).join(' ')}`);
}

console.log('\n[7] 여러 바퀴 겹치기 — record_trajectory.py 의 align_laps 와 같은 결과여야 한다');
{
  // a=2, b=5 타원 한 바퀴는 약 23.0 m.
  const A = 2.0, B = 5.0, CX = 0.1, CY = 4.6;

  // 바퀴마다 점 밀도(=속도), 시작 위상, 반경 노이즈를 다르게 만든다.
  // 실제로 여러 파일에서 읽어 들일 때 서로 안 맞는 조건 전부다.
  const lap = (k, noise, n, phase) => Array.from({ length: n }, (_, i) => {
    const t = 2 * Math.PI * i / n + phase;
    const r = noise * Math.sin(3 * t + k * 2.1);
    return [CX + (A + r) * Math.cos(t), CY + (B + r) * Math.sin(t)];
  });

  const laps = (count, noise, vary) => Array.from({ length: count }, (_, k) =>
    lap(k, noise, vary ? Math.round(1150 * (1 + 0.4 * k)) : 1150,
        vary ? k * 0.7 : 0));

  for (const noise of [0, 0.03, 0.05]) {
    const { pts, spread } = alignLaps(laps(3, noise, true));
    const tol = noise === 0 ? 0.005 : noise * 0.3;
    check(`노이즈 ${(noise * 100).toFixed(0)} cm — 벌어짐이 진폭과 일치`,
      Math.abs(spread - noise) < tol && pts.length === 400,
      `벌어짐 ${(spread * 100).toFixed(1)} cm`);
  }

  check('바퀴 1 개면 그대로 통과', alignLaps(laps(1, 0, false)).spread === 0);

  // 대응이 호길이 비율이 아니라 최근접점이어야 하는 이유:
  // 위상과 주행거리가 다른 동일 곡선의 벌어짐은 0 이어야 한다.
  const shifted = [lap(0, 0, 1150, 0), lap(0, 0, 1400, 1.3), lap(0, 0, 900, 3.7)];
  check('위상·속도만 다른 같은 곡선 -> 벌어짐 0',
    alignLaps(shifted).spread < 0.005,
    `${(alignLaps(shifted).spread * 100).toFixed(2)} cm`);

  // 평균이 한 바퀴보다 참값에 가까운가
  const noisy = laps(3, 0.05, true);
  const truth = (px, py) => {
    let best = Infinity;
    for (let i = 0; i < 2000; i++) {
      const t = 2 * Math.PI * i / 2000;
      best = Math.min(best, Math.hypot(px - (CX + A * Math.cos(t)), py - (CY + B * Math.sin(t))));
    }
    return best;
  };
  const avg = alignLaps(noisy).pts;
  const errAvg = Math.max(...avg.filter((_, i) => i % 20 === 0).map((q) => truth(q[0], q[1])));
  const errOne = Math.max(...noisy[0].filter((_, i) => i % 60 === 0).map((q) => truth(q[0], q[1])));
  check('평균이 한 바퀴보다 참값에 가깝다', errAvg < errOne,
    `평균 ${(errAvg * 100).toFixed(1)} cm  vs  1 바퀴 ${(errOne * 100).toFixed(1)} cm`);

  check('평균에서 씨앗 노드 생성',
    (() => { const n = seedFromTrajectory(avg, 0.5).length; return n > 30 && n < 60; })(),
    `${seedFromTrajectory(avg, 0.5).length} 개`);

  // 옛 형식(한 파일에 여러 바퀴)도 계속 읽혀야 한다
  const joined = laps(3, 0, false).flat();
  joined.push(joined[0]);
  check('단일 파일 여러 바퀴 -> splitLaps 로 3 개', splitLaps(joined).length === 3,
    `${splitLaps(joined).length} 개`);
}

console.log('\n[적응 씨앗] 오차 기준으로 노드를 깐다 — 직진 성기게, 코너 촘촘하게');
{
  // 직선 + 반원이 섞인 폐곡선 (경기장 트랙 모양). 호길이 등간격으로 샘플한다.
  const R = 0.6, STRAIGHT = 4.0;
  const traj = [];
  const step = 0.05;
  for (let x = 0; x < STRAIGHT; x += step) traj.push([x, -R]);
  for (let a = -Math.PI / 2; a < Math.PI / 2; a += step / R) {
    traj.push([STRAIGHT + R * Math.cos(a), R * Math.sin(a)]);
  }
  for (let x = STRAIGHT; x > 0; x -= step) traj.push([x, R]);
  for (let a = Math.PI / 2; a < 3 * Math.PI / 2; a += step / R) {
    traj.push([R * Math.cos(a), R * Math.sin(a)]);
  }

  // 곡선 전체를 촘촘히 편 뒤 궤적의 모든 점에서 최단거리를 잰다.
  // 노드를 궤적 인덱스로 되짚지 않으므로 이게 참값이다.
  const maxDev = (pts) => {
    const poly = [];
    for (const c of buildSegments(pts, true)) {
      for (let i = 0; i < 40; i++) poly.push(deCasteljau(c, i / 40));
    }
    poly.push(poly[0]);
    return Math.max(...traj.map((q) => distToPolyline(q, poly)));
  };

  for (const tol of [0.05, 0.02, 0.01]) {
    const r = seedAdaptive(traj, tol, true, { minSpacing: 0.05 });
    check(`오차 ${(tol * 100).toFixed(0)} cm 이내로 수렴`, !r.blocked && maxDev(r.pts) <= tol,
      `노드 ${r.pts.length} 개  실측 최대 ${(maxDev(r.pts) * 100).toFixed(1)} cm`);
  }

  // 최소 간격에 걸려 목표를 못 맞추면 조용히 넘어가지 않고 blocked 로 알려야 한다
  {
    const r = seedAdaptive(traj, 0.002, true, { minSpacing: 0.3 });
    check('최소 간격에 걸리면 blocked 로 보고한다',
      r.blocked && r.dev > 0.002 && Math.abs(maxDev(r.pts) - r.dev) < 0.01,
      `노드 ${r.pts.length} 개  보고 ${(r.dev * 100).toFixed(1)} cm  실측 ${(maxDev(r.pts) * 100).toFixed(1)} cm`);
  }

  // 직진 구간 노드가 코너 구간보다 성겨야 한다
  {
    const pts = seedAdaptive(traj, 0.02, true).pts;
    const onStraight = pts.filter((p) => p[0] > 0.5 && p[0] < STRAIGHT - 0.5);
    const onCorner = pts.filter((p) => p[0] > STRAIGHT || p[0] < 0);
    const dStr = onStraight.length ? (STRAIGHT - 1) * 2 / onStraight.length : Infinity;
    const dCor = onCorner.length ? 2 * Math.PI * R / onCorner.length : Infinity;
    check('직진 간격 > 코너 간격', dStr > dCor * 1.5,
      `직진 ${dStr.toFixed(2)} m  코너 ${dCor.toFixed(2)} m  (직진 ${onStraight.length} · 코너 ${onCorner.length} 개)`);
  }

  // 등간격 씨앗과 같은 노드 수라면 적응 쪽이 더 정확해야 한다
  {
    const adapt = seedAdaptive(traj, 0.01, true, { minSpacing: 0.05 }).pts;
    const uniform = seedFromTrajectory(traj, lapLen(traj) / adapt.length);
    const da = maxDev(adapt), du = maxDev(uniform);
    check('같은 노드 수에서 등간격보다 정확', da < du,
      `적응 ${(da * 100).toFixed(1)} cm (${adapt.length} 개)  vs  등간격 ${(du * 100).toFixed(1)} cm (${uniform.length} 개)`);
  }
}


// ---------------------------------------------------------------- 세그먼트

console.log('\n[세그먼트] 내가 나눈 구간 — 직선/원호/전이 와 경계에서의 C²');

// 편집기가 UI 에서 읽는 값. 검사에 쓸 허용오차를 못 박아 둔다.
sandbox.document.getElementById('c2Theta').value = '0.05';
sandbox.document.getElementById('c2Kappa').value = '0.002';
sandbox.document.getElementById('kappaLimit').value = '2.0';

function layer(pts, brk, link, closed) {
  return {
    pts, closed: !!closed,
    brk: pts.map((_, i) => (brk || []).includes(i)),
    link: pts.map((_, i) => (link || []).includes(i)),
    visible: true,
  };
}

function kmaxOf(sg) {
  let k = 0;
  for (const c of sg.ctrl) k = Math.max(k, segKappaMax(c));
  return k;
}

function lenOf(sg) {
  let l = 0;
  for (const c of sg.ctrl) l += segLength(c);
  return l;
}

// 곡선이 직선 y=0 에서 얼마나 벗어나는가
function bowOf(sg) {
  let b = 0;
  for (const c of sg.ctrl) {
    for (let i = 0; i <= 40; i++) b = Math.max(b, Math.abs(deCasteljau(c, i / 40)[1]));
  }
  return b;
}

{
  // 경계를 안 찍으면 예전 경로 그대로여야 한다. 예전에 만든 lane_graph.yaml 이
  // 다르게 열리면 그건 회귀다.
  const pts = circle(1.0, 12);
  const L = layer(pts, [], [], true);
  const R = buildLayer(L, {});
  const old = buildSegments(pts, true);
  let same = R.legacy && R.segs.length === old.length;
  for (let i = 0; same && i < old.length; i++) {
    for (let j = 0; j < 6; j++) {
      same = same && Math.abs(R.segs[i].ctrl[0][j][0] - old[i][j][0]) < 1e-12
                  && Math.abs(R.segs[i].ctrl[0][j][1] - old[i][j][1]) < 1e-12;
    }
  }
  check('경계 미지정 -> 예전 방식과 완전히 같다', same, `${old.length} seg`);
}

{
  // 노드 2 개 = 직선. 여기서 조금이라도 휘면 직진 구간을 만들 수 없다.
  const L = layer([[0, 0], [2, 0], [4, 0]], [0, 1, 2], [], false);
  const R = buildLayer(L, {});
  check('노드 2 개 -> 직선', R.segs.length === 2 && R.segs.every((s) => s.type === 'line'),
    R.segs.map((s) => s.type).join(' '));
  check('직선 세그먼트는 정확히 직선', R.segs.every((s) => kmaxOf(s) < 1e-12 && bowOf(s) < 1e-12),
    `max|k| ${Math.max(...R.segs.map(kmaxOf)).toExponential(1)}`);
  check('직선 길이 = 현 길이', Math.abs(lenOf(R.segs[0]) - 2) < 1e-9,
    lenOf(R.segs[0]).toFixed(6));
}

{
  // 노드 3 개 = 그 세 점의 외접원. 회전각을 키워 가며 원에서 안 벗어나는지 본다.
  for (const deg of [30, 60, 90, 120, 150]) {
    const R0 = 0.6, a = deg * Math.PI / 180;
    const pts = [0, a / 2, a].map((t) => [R0 * Math.cos(t), R0 * Math.sin(t)]);
    const L = layer(pts, [0, 2], [], false);
    const B = buildLayer(L, {});
    const sg = B.segs[0];
    const kerr = Math.abs(kmaxOf(sg) - 1 / R0) * R0;      // 상대오차
    const lerr = Math.abs(lenOf(sg) - R0 * a);
    check(`원호 ${deg}도 — 세 점을 지나고 곡률이 1/R`,
      sg.type === 'arc' && sg.dev < 5e-4 && kerr < 5e-3 && lerr < 1e-3,
      `가운데노드 오차 ${(sg.dev * 1000).toFixed(3)} mm · k 상대오차 ${(kerr * 100).toFixed(2)}% · ` +
      `길이오차 ${(lerr * 1000).toFixed(3)} mm`);
  }
}

{
  // 가운데 노드를 밀면 곡률이 그만큼 바뀌어야 한다 — "세 노드로 곡률을 조정한다".
  const p0 = [0, 0], p1 = [2, 0];
  const prev = [];
  for (const h of [0.10, 0.25, 0.50]) {
    const L = layer([p0, [1, h], p1], [0, 2], [], false);
    const sg = buildLayer(L, {}).segs[0];
    // 세 점의 외접원 반지름: R = (a b c) / (4 A)
    const R0 = (Math.hypot(1, h) ** 2 * 2) / (4 * (0.5 * 2 * h));
    prev.push([h, kmaxOf(sg), 1 / R0]);
  }
  check('가운데 노드로 곡률이 직접 정해진다',
    prev.every(([, k, want]) => Math.abs(k - want) / want < 5e-3) &&
    prev[0][1] < prev[1][1] && prev[1][1] < prev[2][1],
    prev.map(([h, k]) => `h${h}->k${k.toFixed(3)}`).join('  '));
}

{
  // 직선 - 원호 - 직선. C² 검증이 위반을 정확히 집어내야 한다.
  const R0 = 0.6, s = R0 * Math.SQRT1_2;
  const pts = [[0, 0], [2, 0], [2 + s, R0 - s], [2 + R0, R0], [2 + R0, R0 + 1.5]];
  const L = layer(pts, [0, 1, 3, 4], [], false);
  const B = buildLayer(L, {});

  check('세그먼트 종류가 노드 수대로 정해진다',
    B.segs.map((x) => x.type).join(',') === 'line,arc,line',
    B.segs.map((x) => x.type).join(','));
  check('직선|원호 경계는 C² 가 아니라고 보고한다',
    B.joints.length === 2 &&
    Math.abs(Math.abs(B.joints[0].dk) - 1 / R0) < 1e-6 &&
    Math.abs(B.joints[0].dth) < 1e-9,
    `Δκ ${B.joints.map((j) => j.dk.toFixed(3)).join(' / ')} · ` +
    `Δθ ${B.joints.map((j) => (j.dth * 180 / Math.PI).toFixed(3)).join(' / ')}°`);

  // 전이 세그먼트를 양쪽에 하나씩 끼우면 전부 C² 가 되어야 한다.
  const pts2 = [[0, 0], [1.7, 0], [2, 0], [2 + s, R0 - s], [2 + R0, R0],
                [2 + R0, R0 + 0.3], [2 + R0, R0 + 1.5]];
  const L2 = layer(pts2, [0, 1, 2, 4, 5, 6], [1, 4], false);
  const B2 = buildLayer(L2, {});

  check('전이 세그먼트를 끼우면 모든 경계가 C²',
    B2.segs.map((x) => x.type).join(',') === 'line,link,arc,link,line' &&
    B2.joints.every((j) => Math.abs(j.dth) < 1e-9 && Math.abs(j.dk) < 1e-9),
    `${B2.joints.length} 경계 · maxΔκ ` +
    Math.max(...B2.joints.map((j) => Math.abs(j.dk))).toExponential(1));
  check('전이를 끼워도 직선은 정확히 직선으로 남는다',
    bowOf(B2.segs[0]) < 1e-12 && kmaxOf(B2.segs[0]) < 1e-12,
    `휨 ${(bowOf(B2.segs[0]) * 1000).toExponential(1)} mm`);
  check('전이 구간의 곡률이 이웃을 넘지 않는다',
    kmaxOf(B2.segs[1]) <= 1 / R0 + 1e-6,
    `전이 max|k| ${kmaxOf(B2.segs[1]).toFixed(4)}  (원호 ${(1 / R0).toFixed(4)})`);

  // 평균으로 강제하면 C² 는 되지만 직선이 휜다 — UI 의 경고와 같은 말인지 확인.
  const B3 = buildLayer(L, { blend: true });
  check('평균 강제는 C² 를 만들지만 직선을 휘게 한다',
    B3.joints.every((j) => j.forced) && bowOf(B3.segs[0]) > 0.01,
    `직선 휨 ${(bowOf(B3.segs[0]) * 1000).toFixed(1)} mm · ` +
    `직선 max|k| ${kmaxOf(B3.segs[0]).toFixed(3)}`);
}

{
  // 닫힌 곡선에서 마지막 세그먼트가 첫 노드로 되감기는지.
  const pts = circle(1.0, 8);
  const L = layer(pts, [0, 2, 4, 6], [], true);
  const B = buildLayer(L, {});
  check('폐곡선은 마지막 세그먼트가 첫 노드로 돌아온다',
    B.segs.length === 4 && B.joints.length === 4 &&
    B.segs[3].idx[B.segs[3].idx.length - 1] === 0,
    `${B.segs.length} 세그먼트 · ${B.segs.map((x) => x.type).join(',')}`);

  const total = B.segs.reduce((a, x) => a + lenOf(x), 0);
  check('폐곡선 원 둘레가 맞는다', Math.abs(total - 2 * Math.PI) < 1e-3,
    `${total.toFixed(4)} m  (참 ${(2 * Math.PI).toFixed(4)})`);
}

{
  // 노드를 넣고 빼도 경계 플래그가 같이 움직여야 한다.
  const L = layer([[0, 0], [1, 0], [2, 0], [3, 0]], [0, 2], [], false);
  nodeInsert(L, 1, [0.5, 0]);
  check('노드 삽입 후 경계가 따라 이동',
    L.pts.length === 5 && L.brk.map((b) => (b ? '1' : '0')).join('') === '10010',
    L.brk.map((b) => (b ? '■' : '·')).join(''));
  nodeRemove(L, 1);
  check('노드 삭제 후 경계 복원',
    L.pts.length === 4 && L.brk.map((b) => (b ? '1' : '0')).join('') === '1010',
    L.brk.map((b) => (b ? '■' : '·')).join(''));

  // 뒤집으면 전이 표시가 반대쪽 노드로 옮겨가야 한다.
  const M = layer([[0, 0], [1, 0], [2, 0], [3, 0]], [0, 1, 2, 3], [1], false);
  const before = buildLayer(M, {}).segs.map((x) => x.type).join(',');
  reverseLayer(M);
  const after = buildLayer(M, {}).segs.map((x) => x.type).join(',');
  check('진행방향을 뒤집어도 세그먼트 구성이 보존된다',
    before === 'line,link,line' && after === 'line,link,line',
    `${before}  ->  ${after}`);
}

function lapLen(p) {
  let s = 0;
  for (let i = 1; i < p.length; i++) s += Math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);
  return s + Math.hypot(p[0][0] - p[p.length - 1][0], p[0][1] - p[p.length - 1][1]);
}

console.log(failed ? `\n실패 ${failed} 건\n` : '\n전부 통과\n');
process.exit(failed ? 1 : 0);
