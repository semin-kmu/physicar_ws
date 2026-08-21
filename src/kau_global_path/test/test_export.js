// 편집기 전 구간 스모크 테스트 — 실제 kau_v3 지도를 읽고, 가상 트랙을 넣고,
// 두 yaml 을 뽑아서 형식과 기하를 검사한다.
//
//   node src/kau_global_path/test/test_export.js
//
// 특히 지키는 것: 내보낸 경계 고리가 두 차로 중심선을 **자르지 않는다.**
// 자르면 자기 차로 위에 선 콘이 ROI 밖으로 떨어진다 (README 4.6).

'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.join(__dirname, '..', '..', '..');
const VALUES = { laneWidth: '0.40', bndSpacing: '0.10', kappaLimit: '2.0', seedSpacing: '0.5' };

const stubEl = (id) => new Proxy({ __id: id }, {
  get(t, k) {
    if (k === 'value') return VALUES[t.__id] ?? '0';
    if (k === 'checked') return true;
    if (k === 'style') return {};
    if (k === 'getBoundingClientRect') return () => ({ width: 900, height: 700, left: 0, top: 0 });
    if (k === 'getContext') {
      return () => new Proxy(
        { createImageData: (w, h) => ({ data: new Uint8ClampedArray(w * h * 4) }) },
        { get: (o, m) => o[m] || (() => {}) });
    }
    if (k === 'querySelector') return () => stubEl();
    if (['appendChild', 'addEventListener', 'setAttribute', 'click'].includes(k)) return () => {};
    return t[k];
  },
  set() { return true; },
});

const sb = {
  document: { getElementById: stubEl, createElement: () => stubEl(), body: stubEl() },
  window: { addEventListener() {}, devicePixelRatio: 1 },
  localStorage: { getItem: () => null, setItem() {} },
  fetch: () => Promise.reject(new Error('no server')),
  setTimeout, clearTimeout, console,
  URL: { createObjectURL: () => '', revokeObjectURL() {} },
  Blob: function () {},
  JSON, Math, Number, Array, String, Object, Promise, Uint8Array, Uint8ClampedArray,
  Infinity, NaN, isNaN, parseFloat, parseInt,
};
sb.globalThis = sb;
vm.createContext(sb);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'web', 'lane_editor.js'), 'utf8'), sb);

let failed = 0;
const check = (name, ok, detail) => {
  console.log(`${ok ? '  통과' : '  실패'}  ${name}${detail ? '   ' + detail : ''}`);
  if (!ok) failed++;
};

// --- 실제 지도를 읽는다 ----------------------------------------------------
const MAPS = path.join(ROOT, 'src', 'kau_localization', 'maps');
const buf = fs.readFileSync(path.join(MAPS, 'kau_v3.pgm'));
sb.setMap('kau_v3', buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength),
  fs.readFileSync(path.join(MAPS, 'kau_v3.yaml'), 'utf8'));

console.log('\n[1] 실제 kau_v3 지도 파싱 + 좌표 왕복');
{
  let worst = 0;
  for (const w of [[0, 0], [1.5, -0.5], [-3.79, 10.0], [3.9, 10.9]]) {
    const back = sb.imageToWorld(sb.worldToImage(w));
    worst = Math.max(worst, Math.hypot(back[0] - w[0], back[1] - w[1]));
  }
  check('world -> image -> world', worst < 1e-9, `최대 오차 ${worst.toExponential(2)} m`);

  const tl = sb.imageToWorld([0, 0]);
  const br = sb.imageToWorld([156, 255]);
  check('y 뒤집기 (README 2.3)', tl[1] > br[1],
    `좌상단 y ${tl[1].toFixed(3)} > 우하단 y ${br[1].toFixed(3)}`);
}

// --- 가상 트랙을 넣는다 ----------------------------------------------------
const CX = 0.1, CY = 4.6;
const oval = (a, b, n) => Array.from({ length: n }, (_, i) => {
  const t = 2 * Math.PI * i / n;
  return [CX + a * Math.cos(t), CY + b * Math.sin(t)];
});

const laneInner = oval(2.0, 5.0, 26);
const laneOuter = oval(2.5, 5.5, 28);
const W = 0.40;
const bndOuter = sb.offsetPolyline(laneOuter, true, W / 2, true);
const bndInner = sb.offsetPolyline(laneInner, true, W / 2, false);

const node = (b) => (p, i) => `    - {id: ${b + i}, x: ${p[0].toFixed(4)}, y: ${p[1].toFixed(4)}}`;
sb.importLaneGraph([
  'lane_graph:', '  nodes:',
  ...laneInner.map(node(0)), ...laneOuter.map(node(100)),
  ...bndOuter.map(node(200)), ...bndInner.map(node(300)),
  '  closed:', '    lane_inner: true', '    lane_outer: true',
  '    bnd_outer: true', '    bnd_inner: true',
].join('\n'));

const files = {};
sb.download = (name, text) => { files[name] = text; };
sb.exportLaneGraph();
sb.exportBoundary();

console.log('\n[2] lane_graph.yaml');
{
  const t = files['lane_graph.yaml'];
  check('생성됨', !!t, t ? `${t.split('\n').length} 줄` : '');
  check('map_source 기록 (README 6.3)', /map_source:\s*kau_v3/.test(t));
  check('routes 가 폐곡선으로 닫힘',
    /inner_loop:\s*\[0,[^\]]*,\s*0\]/.test(t) && /outer_loop:\s*\[100,[^\]]*,\s*100\]/.test(t));
  check('lane_change 자리 안내가 있다', /lane_change/.test(t));
  const ids = [...t.matchAll(/\{id:\s*(\d+)/g)].map((m) => +m[1]);
  check('노드 id 중복 없음', new Set(ids).size === ids.length, `${ids.length} 개`);
  const edges = [...t.matchAll(/\{from:\s*(\d+),\s*to:\s*(\d+)/g)];
  const idset = new Set(ids);
  check('엣지가 전부 존재하는 노드를 가리킴',
    edges.every(([, a, b]) => idset.has(+a) && idset.has(+b)), `${edges.length} 엣지`);
}

console.log('\n[3] kau_v3_track.yaml — Object Detection ROI (README 4.6)');
{
  const t = files['kau_v3_track.yaml'];
  check('생성됨', !!t, t ? `${t.split('\n').length} 줄` : '');

  const arr = (key) => {
    const m = t.match(new RegExp(`${key}:\\s*\\[([\\s\\S]*?)\\]`));
    return m[1].split(',').map((v) => parseFloat(v)).filter(Number.isFinite);
  };
  const ox = arr('track_outer_x'), oy = arr('track_outer_y');
  const ix = arr('track_inner_x'), iy = arr('track_inner_y');

  check('수신측 파라미터 이름 4개',
    /track_outer_x/.test(t) && /track_outer_y/.test(t) &&
    /track_inner_x/.test(t) && /track_inner_y/.test(t));
  check('x / y 길이 일치', ox.length === oy.length && ix.length === iy.length,
    `outer ${ox.length}, inner ${ix.length}`);

  const gaps = (X, Y) => {
    const g = [];
    for (let i = 1; i < X.length; i++) g.push(Math.hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]));
    return [Math.min(...g), Math.max(...g)];
  };
  const [go0, go1] = gaps(ox, oy);
  check('0.10 m 등간격', Math.abs(go0 - 0.1) < 2e-3 && Math.abs(go1 - 0.1) < 2e-3,
    `${go0.toFixed(4)} ~ ${go1.toFixed(4)} m`);

  const area = (X, Y) => Math.abs(X.reduce((a, _, i) => {
    const j = (i + 1) % X.length;
    return a + X[i] * Y[j] - X[j] * Y[i];
  }, 0)) / 2;
  check('outer 가 inner 를 감싼다', area(ox, oy) > area(ix, iy),
    `${area(ox, oy).toFixed(2)} > ${area(ix, iy).toFixed(2)} m^2`);

  // 핵심: 고리가 두 차로를 자르지 않아야 한다
  const inpoly = (X, Y, px, py) => {
    let c = false;
    for (let i = 0, j = X.length - 1; i < X.length; j = i++) {
      if ((Y[i] > py) !== (Y[j] > py) &&
          px < (X[j] - X[i]) * (py - Y[i]) / (Y[j] - Y[i]) + X[i]) c = !c;
    }
    return c;
  };
  for (const [name, lane] of [['안쪽 차로', laneInner], ['바깥쪽 차로', laneOuter]]) {
    const ok = lane.every(([x, y]) => inpoly(ox, oy, x, y) && !inpoly(ix, iy, x, y));
    check(`${name} 중심선이 전부 ROI 고리 안`, ok, `${lane.length} 점`);
  }

  // 벽 밖으로 새지 않는가 (kau_v3 내부 free 범위 안)
  const inside = ox.every((x, i) => x > -3.5 && x < 3.7 && oy[i] > -1.4 && oy[i] < 10.7);
  check('경계가 지도 안에 있다', inside,
    `x [${Math.min(...ox).toFixed(2)}, ${Math.max(...ox).toFixed(2)}] ` +
    `y [${Math.min(...oy).toFixed(2)}, ${Math.max(...oy).toFixed(2)}]`);
}

console.log(failed ? `\n실패 ${failed} 건\n` : '\n전부 통과\n');
process.exit(failed ? 1 : 0);
