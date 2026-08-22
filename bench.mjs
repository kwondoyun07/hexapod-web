// node bench.mjs [초] — 브라우저 없이 물리를 돌려 걸음을 측정한다.
// 렌더 없이 robot.js / gait.js 를 그대로 쓰므로, 여기서 나온 수치가 곧 브라우저 거동이다.
// 파라미터 스윕은 브라우저 콘솔보다 여기가 훨씬 빠르다.
import * as THREE from 'three';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { SPEC, buildGround, buildHexapod } from './robot.js';
import { GAIT, makeTargets, footTargets } from './gait.js';
import { pathToFileURL } from 'node:url';

await RAPIER.init();

export const DEFAULT_ENV = { friction: 2.0, hz: 480, lengthUnit: 0.1, pgs: 8, gravity: -9.81 };
export const DEFAULT_SERVO = { stiffness: 900, damping: 0.3, maxForce: 6.0 };

/**
 * 한 번 걸어보고 지표를 돌려준다.
 * @param gait  GAIT 에 덮어씌울 값 (vx/omega/stance/height/...)
 * @param opts  { secs, warm, servo, env }
 */
export function run(gait = {}, opts = {}) {
  const { secs = 5, warm = 1.5 } = opts;
  const servo = { ...DEFAULT_SERVO, ...opts.servo };
  const env = { ...DEFAULT_ENV, ...opts.env };
  const G = { ...GAIT, ...gait };

  const world = new RAPIER.World({ x: 0, y: env.gravity, z: 0 });
  world.timestep = 1 / env.hz;
  world.numSolverIterations = 8;
  world.integrationParameters.lengthUnit = env.lengthUnit;
  world.integrationParameters.numInternalPgsIterations = env.pgs;
  buildGround(world, env.friction);

  const scene = new THREE.Scene(); // 렌더는 없지만 buildHexapod 가 메시를 여기 담는다
  const robot = buildHexapod(world, scene, { height: G.height, reach: G.stance });
  const foot = makeTargets();

  const q = new THREE.Quaternion();
  const yawOf = () => {
    const r = robot.bodyRb.rotation();
    return Math.atan2(2 * (r.w * r.y + r.x * r.z), 1 - 2 * (r.y * r.y + r.z * r.z));
  };
  const tiltOf = () => {
    const r = robot.bodyRb.rotation();
    return (Math.acos(Math.max(-1, Math.min(1, 1 - 2 * (r.x * r.x + r.z * r.z)))) * 180) / Math.PI;
  };

  // main.js 의 렌더 루프와 같은 구조로 돌린다: 프레임(1/60초)마다 gate 를 올리고
  // 그 안에서 hz/60 번 물리를 밟는다. gate 를 빼면 출발 편향이 그대로 나와
  // 브라우저 거동과 어긋난다 (측면 이탈이 13 mm → 290 mm 로 벌어졌다).
  const base = { stepLen: G.stepLen, stepHeight: G.stepHeight, push: G.push };
  const commanded = Math.abs(G.vx) + Math.abs(G.vz) + Math.abs(G.omega) > 1e-9;
  const perFrame = Math.max(1, Math.round(env.hz / 60));
  let t = 0;
  let gate = 0;
  const frame = (n = 1) => {
    for (let f = 0; f < n; f++) {
      gate += ((commanded ? 1 : 0) - gate) * 0.06;
      G.stepLen = base.stepLen * gate;
      G.stepHeight = base.stepHeight * gate;
      G.push = base.push * gate;
      for (let k = 0; k < perFrame; k++) {
        footTargets(t, SPEC, G, foot);
        robot.drive(foot, servo);
        world.step();
        t += world.timestep;
      }
    }
  };

  frame(Math.round(warm * 60));
  const p0 = robot.bodyRb.translation();
  const start = { x: p0.x, z: p0.z, yaw: yawOf() };
  const ys = [];
  const tilts = [];
  const N = Math.round(secs * 60);
  for (let i = 0; i < N; i++) {
    frame();
    ys.push(robot.bodyRb.translation().y);
    tilts.push(tiltOf());
  }
  const p = robot.bodyRb.translation();
  const mean = (a) => a.reduce((s, v) => s + v, 0) / a.length;
  const m = mean(ys);
  let cross = 0;
  for (let i = 1; i < ys.length; i++) if ((ys[i - 1] - m) * (ys[i] - m) < 0) cross++;
  const dyaw = ((yawOf() - start.yaw) * 180) / Math.PI;

  const out = {
    dx: +(p.x - start.x).toFixed(3),
    dz: +(p.z - start.z).toFixed(3),
    spd: +((p.x - start.x) / secs).toFixed(3),
    dyaw: +dyaw.toFixed(1),
    y: +m.toFixed(4),
    yPP_mm: +((Math.max(...ys) - Math.min(...ys)) * 1000).toFixed(1),
    hz: +(cross / 2 / secs).toFixed(1),
    tiltMax: +Math.max(...tilts).toFixed(2),
    fell: Math.max(...tilts) > 45,
  };
  robot.dispose();
  world.free();
  return out;
}

/** 이론상 몸통 속도 = 보폭 / stance 시간. 효율 1.0 이면 발이 안 미끄러진 것. */
export const theorySpeed = (g) => g.stepLen / (g.duty * g.period);

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const secs = Number(process.argv[2]) || 5;
  const cases = {
    stand: { stepLen: 0, stepHeight: 0, push: 0, vx: 0 },
    forward: {},
    back: { vx: -1 },
    strafe: { vx: 0, vz: 1 },
    turn: { vx: 0, omega: 10 },
  };
  console.log(`총질량 ${(SPEC.mass.body + 6 * (SPEC.mass.coxa + SPEC.mass.femur + SPEC.mass.tibia + SPEC.mass.foot)).toFixed(2)} kg`);
  for (const [name, over] of Object.entries(cases)) {
    const r = run(over, { secs });
    const eff = name === 'forward' ? `  eff=${(r.spd / theorySpeed({ ...GAIT, ...over })).toFixed(2)}` : '';
    console.log(
      `${name.padEnd(8)} dx=${String(r.dx).padStart(6)} dz=${String(r.dz).padStart(6)} yaw=${String(r.dyaw).padStart(6)}°` +
        ` y=${r.y} yPP=${String(r.yPP_mm).padStart(5)}mm ${String(r.hz).padStart(4)}Hz tilt=${r.tiltMax}°${r.fell ? '  넘어짐' : ''}${eff}`
    );
  }
}
