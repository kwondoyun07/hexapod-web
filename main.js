import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { SPEC, buildGround, buildHexapod } from './robot.js';
import { GAIT, makeTargets, footTargets } from './gait.js';
import { makeStick, keysOf } from './joystick.js';

await RAPIER.init();

// 실물 PhantomX 메시 (BSD-2, assets/README.md 참고). 물리는 여전히 primitive collider 로 돈다 —
// 이건 visual 전용이다. 못 받아오면 박스로 그리고 시뮬은 그대로 돌아간다.
let PARTS = null;
try {
  const draco = new DRACOLoader();
  draco.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.185.1/examples/jsm/libs/draco/');
  const loader = new GLTFLoader();
  loader.setDRACOLoader(draco);
  const gltf = await loader.loadAsync('./assets/phantomx.glb');
  draco.dispose();
  PARTS = {};
  gltf.scene.traverse((o) => {
    if (o.isMesh) PARTS[o.name.split('.')[0]] = o;
  });
} catch (err) {
  console.warn('메시를 못 읽어 primitive 로 그린다:', err);
}

const G = { ...GAIT };
// 서보 흉내 (ForceBased). stiffness = N·m/rad, damping = N·m·s/rad, maxForce = N·m.
// maxForce 가 실물 서보의 토크 한계 그 자체다 — 예컨대 MG996R 급이면 1~1.2 로 낮춰보라.
// "시뮬에선 걷는데 실물은 못 버티는" 상황이 여기서 먼저 드러난다.
// damping 을 키우면 오히려 나빠진다. 링크 관성이 작아 과감쇠 쪽이 솔버를 흔든다 —
// 0.3 근처가 실측 최적이고, 3 까지 올리면 전진이 음수로 뒤집힌다.
const SERVO = { stiffness: 900, damping: 0.3, maxForce: 6.0 };
// 지면 마찰 2.0 은 고무 발 + 매트 수준이다. 1.0 으로 낮추면 보폭 효율이
// 0.80 → 0.68 로 떨어진다 — 실제 발 재질에 맞춰 조정하라.
// hz = 물리 적분 주파수. 240 Hz 로는 몸통이 20~30 mm 떨고, 480 Hz 면 2.4 mm 로 잡힌다.
// 960 Hz 는 계산량만 2배일 뿐 480 보다 나아지지 않았다.
// 프레임당 스텝 수는 hz/60 으로 자동 계산되므로 실시간 배속은 항상 1배다.
const ENV = {
  friction: 2.0, hz: 480,
  // Rapier 는 1 m 스케일을 가정한다. 이 로봇은 0.3 m 크기라 기본값이면 허용 관통이
  // 1 mm — 발 반지름 16 mm 에 비해 너무 커서 발이 매 스텝 들락거리고, 그게 14 Hz
  // 짜리 떨림이 된다. lengthUnit 을 로봇 스케일로 낮추면 사라진다.
  lengthUnit: 0.1,
  // 접촉 솔버 내부 반복 (기본 1). 다리 6개가 동시에 지면을 미는 구조라 1 로는 모자란다.
  // 4 → 몸통 진동 2.5 mm, 8 → 1.2 mm. 실측 비용은 60 fps 예산의 12% → 15% 뿐이라
  // 8 을 쓴다. 프레임이 모자라면 낮춰라 (2 까지는 9%).
  pgs: 8,
};
// 조이스틱이 매 프레임 G.vx/vz/omega/stepLen/height 를 덮어쓴다.
// 슬라이더는 G 가 아니라 이 기준값을 만진다 — 스틱을 끝까지 밀었을 때의 값이다.
const CTRL = {
  stepLen: 0.07, stepHeight: 0.06, push: 0.006, height: 0.115, heightRange: 0.03,
  // 선회 명령을 병진과 같은 스케일로 올린다. 접선 stride 는 omega × (mount 반경) 이라
  // 게인이 없으면 회전 기여가 병진의 1/8 밖에 안 되고, 전진 중 선회가 무시된다.
  turnGain: 1 / Math.hypot(SPEC.legs[0].x, SPEC.legs[0].z),
};
let gate = 0; // 걸음 on/off 를 시간축에서 부드럽게 (계단으로 켜면 몸통이 30 mm 튄다)
// 몸통 높이는 스틱보다 훨씬 느리게 따라간다. 스틱 속도로 3 cm 를 0.1 초 만에 명령하면
// 서보가 급하게 밀어올려 16 mm 오버슈트한다 — 눈에는 '붕 뜨는' 것으로 보인다.
let heightState = CTRL.height;

// ---------- 씬 ----------
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c1419);
scene.fog = new THREE.Fog(0x0c1419, 2.5, 9);

const camera = new THREE.PerspectiveCamera(50, 1, 0.02, 100);
camera.position.set(0.55, 0.4, 0.7);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0.12, 0);
controls.enableDamping = true;
controls.minDistance = 0.25;
controls.maxDistance = 4;

scene.add(new THREE.HemisphereLight(0x9fd4e8, 0x18222a, 1.0));
const sun = new THREE.DirectionalLight(0xffffff, 1.7);
sun.castShadow = true;
sun.shadow.mapSize.set(1024, 1024);
Object.assign(sun.shadow.camera, { left: -0.7, right: 0.7, top: 0.7, bottom: -0.7, near: 0.5, far: 5 });
sun.shadow.camera.updateProjectionMatrix();
scene.add(sun, sun.target);

scene.add(new THREE.GridHelper(14, 140, 0x3f6d80, 0x1c2f39));
const shadowCatcher = new THREE.Mesh(
  new THREE.PlaneGeometry(40, 40),
  new THREE.ShadowMaterial({ opacity: 0.4 })
);
shadowCatcher.rotation.x = -Math.PI / 2;
shadowCatcher.receiveShadow = true;
scene.add(shadowCatcher);

// ---------- 물리 ----------
let world = null;
let robot = null;
let t = 0;
const foot = makeTargets();

function reset() {
  if (robot) robot.dispose();
  if (world) world.free();
  world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  world.timestep = 1 / ENV.hz;
  world.numSolverIterations = 8;
  world.integrationParameters.lengthUnit = ENV.lengthUnit;
  world.integrationParameters.numInternalPgsIterations = ENV.pgs;
  buildGround(world, ENV.friction);
  robot = buildHexapod(world, scene, { height: G.height, reach: G.stance }, PARTS);
  t = 0;
  const p = robot.bodyRb.translation();
  controls.target.set(p.x, p.y, p.z);
}
reset();

// ---------- UI ----------
// [대상 객체, 키, 라벨, min, max, step, 리셋필요?]
const PARAMS = [
  [CTRL, 'height', '몸통 높이 (m)', 0.05, 0.2, 0.001, false],
  [G, 'stance', '다리 뻗음 (m)', 0.09, 0.24, 0.001, false],
  [CTRL, 'stepLen', '보폭 (m)', 0, 0.14, 0.001, false],
  [CTRL, 'stepHeight', '스윙 높이 (m)', 0, 0.1, 0.001, false],
  [G, 'period', '주기 (s)', 0.2, 2, 0.01, false],
  [G, 'duty', 'duty (접지 비율)', 0.4, 0.85, 0.01, false],
  [CTRL, 'push', '발 하중 깊이 (m)', 0, 0.03, 0.001, false],
  [CTRL, 'turnGain', '선회 게인', 1, 30, 0.5, false],
  [SERVO, 'stiffness', '서보 강성 (N·m/rad)', 20, 2000, 20, false],
  [SERVO, 'damping', '서보 감쇠 (0.5 초과 = 발산)', 0.02, 0.8, 0.02, false],
  [SERVO, 'maxForce', '서보 최대 토크 (N·m)', 0.5, 20, 0.5, false],
  [ENV, 'friction', '지면 마찰', 0.1, 2, 0.05, true],
  [ENV, 'hz', '물리 주파수 (Hz)', 120, 1920, 120, true],
  [ENV, 'pgs', '접촉 솔버 반복', 1, 12, 1, true],
];

const ui = document.getElementById('params');
for (const [obj, key, label, min, max, step, needsReset] of PARAMS) {
  const row = document.createElement('label');
  row.className = 'row';
  row.innerHTML = `<span>${label}${needsReset ? ' <em>↻</em>' : ''}</span><input type="range" min="${min}" max="${max}" step="${step}"><b></b>`;
  const [input, out] = [row.querySelector('input'), row.querySelector('b')];
  input.value = obj[key];
  out.textContent = (+obj[key]).toFixed(step < 0.01 ? 3 : step < 1 ? 2 : 0);
  input.oninput = () => {
    obj[key] = +input.value;
    out.textContent = (+input.value).toFixed(step < 0.01 ? 3 : step < 1 ? 2 : 0);
    if (needsReset) reset();
  };
  ui.append(row);
}
document.getElementById('reset').onclick = reset;

// ---------- 조종 ----------
const KEYMAP_L = { up: 'KeyW', down: 'KeyS', left: 'KeyA', right: 'KeyD' };
const KEYMAP_R = { up: 'KeyI', down: 'KeyK', left: 'KeyJ', right: 'KeyL' };
const stickL = makeStick(document.getElementById('stickL'), KEYMAP_L, 'WASD');
const stickR = makeStick(document.getElementById('stickR'), KEYMAP_R, 'IJKL');
const STICK_KEYS = keysOf(KEYMAP_L, KEYMAP_R);
const pressed = new Set();

addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return; // 슬라이더에 포커스가 있으면 슬라이더 몫
  if (STICK_KEYS.has(e.code)) {
    pressed.add(e.code);
    e.preventDefault();
  }
});
addEventListener('keyup', (e) => pressed.delete(e.code));
addEventListener('blur', () => pressed.clear()); // 창을 벗어날 때 키가 눌린 채 남지 않게

function applySticks() {
  const l = stickL.update(pressed);
  const r = stickR.update(pressed);
  const move = Math.min(1, Math.hypot(l.x, l.y));
  const turn = Math.abs(r.x);
  G.vx = -l.y; // W(위) = 전진
  G.vz = l.x; // D = 로봇 오른쪽(+Z)
  G.omega = -r.x * CTRL.turnGain; // J = 좌선회(반시계)
  // 보폭은 두 스틱 중 큰 쪽을 따른다. move 만 쓰면 선회(오른쪽 스틱)일 때
  // 보폭이 0 이 되어 발이 제자리걸음만 하고 로봇이 돌지 않는다.
  // gate 를 곱해 첫 스텝을 작게 시작한다 — tripod 출발은 좌2/우1 이 먼저 나가서
  // 본질적으로 비대칭이라, 처음부터 최대 보폭으로 내디디면 몸이 7.5° 틀어진 채 출발한다.
  G.stepLen = CTRL.stepLen * Math.min(1, Math.max(move, turn)) * gate;
  const wantHeight = CTRL.height - r.y * CTRL.heightRange; // I(위) = 몸을 높인다
  heightState += (wantHeight - heightState) * 0.03;
  G.height = heightState;
  // 명령이 없으면 발을 멈추고 하중도 뺀다 (서 있는 동안 push 를 걸면 몸통이 떤다).
  // 다만 켜고 끄기를 계단으로 하면 그 순간 로봇이 튀어오르므로 시간축에서 램프한다.
  gate += ((move + turn > 0.02 ? 1 : 0) - gate) * 0.06;
  G.stepHeight = CTRL.stepHeight * gate;
  G.push = CTRL.push * gate;
  return { move, turn };
}

const hud = document.getElementById('hud');
const e = new THREE.Euler();
const q = new THREE.Quaternion();

// ---------- 루프 ----------
const bodyPos = new THREE.Vector3();
let frames = 0;
let fpsMark = performance.now();
let fps = 0;

function frame() {
  requestAnimationFrame(frame);

  // 서보 목표는 매 물리 스텝마다 갱신한다. 프레임당 1회만 갱신하면 목표가 계단식이 되어
  // 스윙 발(0.2 m/s)이 궤적을 32 mm 나 뒤처진다 — 게인을 10배 올려도 안 줄어든다.
  applySticks();
  const n = Math.max(1, Math.round(ENV.hz / 60));
  for (let k = 0; k < n; k++) {
    footTargets(t, SPEC, G, foot);
    robot.drive(foot, SERVO);
    world.step();
    t += world.timestep;
  }
  robot.sync();

  const p = robot.bodyRb.translation();
  bodyPos.set(p.x, p.y, p.z);
  const d = bodyPos.clone().sub(controls.target).multiplyScalar(0.08);
  controls.target.add(d);
  camera.position.add(d); // target 만 옮기면 시점 각도가 틀어진다
  sun.position.set(p.x + 1.1, p.y + 1.9, p.z + 0.8);
  sun.target.position.copy(bodyPos);

  controls.update();
  renderer.render(scene, camera);

  if (++frames >= 15) {
    const now = performance.now();
    fps = (frames * 1000) / (now - fpsMark);
    frames = 0;
    fpsMark = now;
    const v = robot.bodyRb.linvel();
    const r = robot.bodyRb.rotation();
    e.setFromQuaternion(q.set(r.x, r.y, r.z, r.w), 'YXZ');
    hud.textContent =
      `높이 ${p.y.toFixed(3)} m   속도 ${Math.hypot(v.x, v.z).toFixed(3)} m/s   ` +
      `roll ${((e.z * 180) / Math.PI).toFixed(1)}°  pitch ${((e.x * 180) / Math.PI).toFixed(1)}°   ` +
      `${fps.toFixed(0)} fps`;
  }
}

// 개발용 훅. 콘솔에서 파라미터를 만지거나, 화면 없이 물리만 돌려 검증할 때 쓴다.
// (탭이 백그라운드면 rAF 가 멈추므로 headless 가 유일한 검증 경로다.
//  나중에 gait 파라미터 그리드 서치를 돌린다면 여기가 진입점이다.)
window.sim = {
  G, SERVO, ENV, reset,
  get body() { return robot.bodyRb; },
  get robot() { return robot; },
  get world() { return world; },
  SPEC, CTRL,
  sticks: { get L() { return stickL; }, get R() { return stickR; }, pressed, apply: applySticks },
  get targets() { return foot; },
  /**
   * dur 초를 정확히 진행한다. 렌더 루프와 동일하게 매 스텝 서보 목표를 갱신한다.
   */
  headless(dur) {
    const total = Math.max(1, Math.round(dur * ENV.hz));
    for (let k = 0; k < total; k++) {
      footTargets(t, SPEC, G, foot);
      robot.drive(foot, SERVO);
      world.step();
      t += world.timestep;
    }
    robot.sync();
    const p = robot.bodyRb.translation();
    const v = robot.bodyRb.linvel();
    const r = robot.bodyRb.rotation();
    e.setFromQuaternion(q.set(r.x, r.y, r.z, r.w), 'YXZ');
    return {
      t, x: p.x, y: p.y, z: p.z,
      speed: Math.hypot(v.x, v.z),
      rollDeg: (e.z * 180) / Math.PI,
      pitchDeg: (e.x * 180) / Math.PI,
    };
  },
};

function resize() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
new ResizeObserver(resize).observe(canvas);
resize();
frame();
