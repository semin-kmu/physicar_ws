// 변형 경로 비교 오버레이 검증. ROS 도 브라우저도 필요 없다.
//
//   node src/kau_global_path/test/test_compare.js
//
// 편집기가 config/ 의 **발행용 yaml 을 그대로** 읽어 겹쳐 그리는 기능이다
// (lane_editor.js 의 CMP_*, README 4.7). 여기서 보는 것은 두 가지다.
//
//   [1] yaml 에서 뽑은 제어점이 check_path.py 가 재는 값과 같은가
//   [2] 콘 sdf 의 sim pose 가 트랙과 같은 변환으로 map 에 올라오는가
//
// 편집기는 DOM 스크립트라 import 가 안 된다. test_geometry.js 와 같은 방식으로
// vm 컨텍스트에 stub 을 깔고 통째로 실행한 뒤 전역을 꺼내 쓴다. fetch 는
// 파일에서 읽어 주는 것으로 바꿔 끼운다 — 서버를 띄우지 않기 위해서다.

'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const WEB = path.join(__dirname, '..', 'web');

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
const DOM_DEFAULTS = {
  vehL: '0.18', vehSteer: '20', vehMargin: '0.90', kappaLimit: '1.8199',
  laneWidth: '0.40', bndSpacing: '0.10', seedSpacing: '0.5', seedTol: '0.02',
  seedMin: '0.15', c2Theta: '1.0', c2Kappa: '0.05',
  cadR: '0.55', cadRcap: '1.5', cadTrans: '0.15',
  imgCx: '0', imgCy: '0', imgW: '1', imgRot: '0', imgAlpha: '0.6',
  trkOx: '3.68', trkOy: '1.39', trkRot: '0', trkAlpha: '0.85',
  dispOx: '3.68', dispOy: '-1.39',
  coneR: '0.09', coneClear: '0.24',
};

// 편집기의 상대경로(web/ 기준)를 실제 파일로 바꿔 준다.
function fakeFetch(url) {
  const file = path.resolve(WEB, url);
  if (!fs.existsSync(file)) {
    return Promise.resolve({ ok: false, status: 404, text: () => Promise.resolve('') });
  }
  const text = fs.readFileSync(file, 'utf8');
  return Promise.resolve({ ok: true, status: 200, text: () => Promise.resolve(text) });
}

const sandbox = {
  document: {
    getElementById: (id) => {
      if (!els[id]) {
        els[id] = stubEl();
        if (DOM_DEFAULTS[id] !== undefined) els[id].value = DOM_DEFAULTS[id];
      }
      return els[id];
    },
    createElement: () => stubEl(),
    body: stubEl(),
  },
  window: { addEventListener() {}, devicePixelRatio: 1 },
  localStorage: { getItem: () => null, setItem() {} },
  fetch: fakeFetch,
  setTimeout, clearTimeout, console,
  URL: { createObjectURL: () => '', revokeObjectURL() {} },
  Blob: function () {},
  JSON, Math, Number, Array, String, Object, Promise, Uint8Array,
  Set, Map, RegExp, isFinite,
  Infinity, NaN, isNaN, parseFloat, parseInt,
};
sandbox.globalThis = sandbox;

vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(WEB, 'lane_editor.js'), 'utf8'), sandbox);

const { parseBezierLayer, loadCompare, loadCones, trackToMap } = sandbox;

// const 로 선언된 것은 vm 컨텍스트의 globalThis 에 안 올라간다. 따로 꺼낸다
// (test_geometry.js 와 같은 사정이다).
const CMP_DEFS = vm.runInContext('CMP_DEFS', sandbox);
const CMP = vm.runInContext('CMP', sandbox);

let failed = 0;

function check(name, ok, detail) {
  console.log(`${ok ? '  통과' : '  실패'}  ${name}${detail ? '   ' + detail : ''}`);
  if (!ok) failed++;
}

// check_path.py 가 같은 파일에서 재는 값. 여기가 어긋나면 편집기가 발행되는
// 곡선과 다른 것을 그리고 있다는 뜻이다.
const EXPECT = {
  lane_graph: { segs: 50, len: 28.872, kmax: 1.8104 },
  right_bias: { segs: 172, len: 29.487, kmax: 1.8094 },
  last_obstacle: { segs: 173, len: 28.866, kmax: 1.8098 },
  every_obstacle: { segs: 172, len: 28.883, kmax: 1.8102 },
};

async function main() {
  check('parseBezierLayer 가 있다', typeof parseBezierLayer === 'function');

  console.log('\n[1] 발행용 yaml 에서 제어점 읽기');
  for (const def of CMP_DEFS) {
    const L = await loadCompare(def);
    const e = EXPECT[def.id];
    if (L.err) {
      check(`${def.id}`, false, L.err);
      continue;
    }
    const shape = L.segs.every((c) => c.length === 6 && c.every((q) => q.length === 2));
    check(`${def.id} — 조각 ${L.segs.length} · ${L.len.toFixed(3)} m · |κ|max ${L.kmax.toFixed(4)}`,
      shape && L.segs.length === e.segs
        && Math.abs(L.len - e.len) < 0.002
        && Math.abs(L.kmax - e.kmax) < 0.002,
      `기대 ${e.segs} / ${e.len} m / ${e.kmax}`);
  }

  console.log('\n[2] 조각이 이어져 있는가 (앞 조각 끝 = 뒤 조각 시작)');
  for (const def of CMP_DEFS) {
    const L = CMP.lanes[def.id];
    if (!L || L.err) continue;
    let worst = 0;
    for (let i = 0; i < L.segs.length; i++) {
      const a = L.segs[i][5];
      const b = L.segs[(i + 1) % L.segs.length][0];
      worst = Math.max(worst, Math.hypot(a[0] - b[0], a[1] - b[1]));
    }
    check(`${def.id} 이음새`, worst < 1e-6, `최대 ${(worst * 1000).toFixed(6)} mm`);
  }

  console.log('\n[3] 콘 sdf -> map 프레임');
  const cones = await loadCones();
  check('콘 6 개를 읽었다', cones.length === 6, `${cones.length} 개`);

  // sim -> map 은 트랙과 같은 변환이다: map_x = ox - sim_y, map_y = sim_x - oy.
  // 값은 gen_lane_variants.py 가 찍는 표와 같아야 한다.
  const EXPECT_MAP = {
    cone1: [-0.161, 8.922], cone2: [2.641, 6.935], cone3: [-0.755, 6.771],
    cone4: [2.288, 8.344], cone5: [-2.128, 2.786], cone6: [-1.091, -0.188],
  };
  const toMap = trackToMap;
  let worst = 0;
  let at = '';
  for (const c of cones) {
    const q = toMap(c.sim);
    const e = EXPECT_MAP[c.name];
    const d = Math.hypot(q[0] - e[0], q[1] - e[1]);
    if (d > worst) { worst = d; at = c.name; }
  }
  check('콘 좌표가 생성기와 같다', worst < 0.001,
    `최대 차이 ${(worst * 1000).toFixed(2)} mm (${at})`);

  console.log('\n[4] 그리기 — 예외 없이 도는가');
  // 지도가 있어야 좌표 변환이 돈다. 실제 kau_v3 값과 같은 모양으로 채운다.
  vm.runInContext(
    'S.map = { ox: -3.793323, oy: -1.764733, res: 0.05, w: 143, h: 243, canvas: null };'
    + 'S.view = { scale: 3, tx: 0, ty: 0 };', sandbox);
  CMP.conesOn = true;
  for (const def of CMP_DEFS) CMP.on[def.id] = true;
  let drew = true;
  try {
    sandbox.drawCompare();
    sandbox.drawCones();
  } catch (e) {
    drew = false;
    check('drawCompare / drawCones', false, String(e.message || e));
  }
  if (drew) check('drawCompare / drawCones 가 네 경로 + 콘을 그린다', true,
    `경로 ${CMP_DEFS.length} · 콘 ${CMP.cones.length}`);

  console.log(failed ? `\n실패 ${failed} 건\n` : '\n전부 통과\n');
  process.exit(failed ? 1 : 0);
}

main();
