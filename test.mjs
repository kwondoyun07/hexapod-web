// node test.mjs — 브라우저 없이 도는 기구학/걸음새 자체 검증.
// 물리(Rapier)는 여기서 안 돈다. 여기가 깨지면 브라우저에서도 반드시 깨진다.
import assert from 'node:assert/strict';
import { legIK, legFK, toLegLocal, toBodyFrame } from './ik.js';
import { GAIT, makeTargets, footTargets, legPhase, shape } from './gait.js';

// PhantomX 실측 (robot.js SPEC 과 같은 값 — 여기서는 three.js 의존성을 피하려고 다시 적는다)
const L = { coxa: 0.054, femur: 0.0661, tibia: 0.1632 };
const spec = {
  legs: [
    { x: 0.1248, z: 0.0616, yaw: -45 },
    { x: 0.0, z: 0.1034, yaw: -90 },
    { x: -0.1248, z: 0.0616, yaw: -135 },
    { x: -0.1248, z: -0.0616, yaw: 135 },
    { x: 0.0, z: -0.1034, yaw: 90 },
    { x: 0.1248, z: -0.0616, yaw: 45 },
  ].map((l) => ({ ...l, yaw: (l.yaw * Math.PI) / 180 })),
};
const REACH_MAX = L.femur + L.tibia;

// 1. IK → FK 왕복이 정확히 닫히는가
let worst = 0;
for (let k = 0; k < 5000; k++) {
  const yaw = (Math.random() - 0.5) * 1.6;
  const reach = L.coxa + 0.06 + Math.random() * 0.15;
  const h = -(0.04 + Math.random() * 0.18);
  const D = Math.hypot(reach - L.coxa, h);
  if (D > REACH_MAX * 0.98 || D < Math.abs(L.femur - L.tibia) * 1.02) continue;
  const p = [reach * Math.cos(yaw), h, reach * Math.sin(yaw)];
  const q = legIK(p[0], p[1], p[2], L);
  const f = legFK(q[0], q[1], q[2], L);
  worst = Math.max(worst, Math.hypot(f[0] - p[0], f[1] - p[1], f[2] - p[2]));
}
assert.ok(worst < 1e-9, `IK/FK 왕복 오차가 크다: ${worst}`);

// 2. 도달 불가 목표에서도 NaN 이 아니라 "최대한 뻗은 자세"가 나오는가
for (const p of [[2, -0.05, 0], [0.001, -0.001, 0], [0.05, -1, 0]]) {
  const q = legIK(p[0], p[1], p[2], L);
  assert.ok(q.every(Number.isFinite), `IK 가 NaN 을 냈다: ${p}`);
  assert.ok(q[2] <= 1e-12, `무릎이 반대로 꺾였다: q3=${q[2]}`);
}

// 3. tripod 불변식: duty 0.5 면 언제나 정확히 3개가 지면에 있다.
//    기본값(duty 0.8)이 아니라 정통 tripod 를 명시해야 하는 검증이다.
const g = { ...GAIT, duty: 0.5 };
for (let n = 0; n < 500; n++) {
  const t = Math.random() * 20;
  let ground = 0;
  for (let i = 0; i < 6; i++) if (shape(legPhase(t, i, g), g)[1] <= 0) ground++;   // stance = 들리지 않음
  assert.equal(ground, 3, `t=${t} 에서 지면 접촉 다리가 ${ground}개`);
}

// 4. duty 를 올리면 접촉 다리가 줄지 않는다 (안정성 방향이 뒤집히지 않는가)
for (const duty of [0.5, 0.65, 0.8]) {
  const gd = { ...GAIT, duty };
  let min = 6;
  for (let n = 0; n < 400; n++) {
    const t = Math.random() * 20;
    let ground = 0;
    for (let i = 0; i < 6; i++) if (shape(legPhase(t, i, gd), gd)[1] <= 0) ground++;
    min = Math.min(min, ground);
  }
  assert.ok(min >= 3, `duty=${duty} 에서 최소 접촉이 ${min}개로 떨어진다`);
}

// 5. 궤적 자체: stance 는 정확히 지면 높이, swing 은 최대 stepHeight
const out = makeTargets();
let maxLift = 0;
for (let n = 0; n <= 400; n++) {
  const t = (n / 400) * g.period;
  footTargets(t, spec, g, out);
  for (let i = 0; i < 6; i++) {
    const lift = out[i][1] + g.height;
    // stance 는 push 만큼 지면 아래를 목표한다 (하중을 싣기 위해) — 그 이상은 안 된다
    assert.ok(lift >= -(g.push || 0) - 1e-12, `발이 push 보다 깊게 파고든다: ${lift}`);
    assert.ok(lift <= g.stepHeight + 1e-12, `스윙이 stepHeight 를 넘는다: ${lift}`);
    maxLift = Math.max(maxLift, lift);
    const D = Math.hypot(Math.hypot(out[i][0], out[i][2]) - L.coxa, out[i][1]);
    assert.ok(D < REACH_MAX * 0.97, `기본 파라미터인데 다리가 닿지 않는다: D=${D}`);
  }
}
assert.ok(Math.abs(maxLift - g.stepHeight) < 1e-3, `스윙 최고점이 stepHeight 에 못 미친다: ${maxLift}`);

// 6. 순수 회전이면 발은 몸통 중심 기준 원 위를 돌아야 한다 — 반경이 변하면 안 된다.
//    이게 깨지면 로봇이 제자리 선회 중에 스스로를 끌고 간다.
//    (mount 기준이 아니라 몸통 중심 기준이다: mount 방위와 다리 방향은 서로 다르다.)
{
  const gr = { ...GAIT, vx: 0, vz: 0, omega: 1 };
  const radii = [];
  for (const time of [0, 0.2, 0.45, 0.7]) {
    footTargets(time, spec, gr, out);
    for (let i = 0; i < 6; i++) {
      const { x: mx, z: mz, yaw } = spec.legs[i];
      const [rx, rz] = toBodyFrame(out[i][0], out[i][2], yaw);
      radii[i] = radii[i] || [];
      radii[i].push(Math.hypot(mx + rx, mz + rz));
    }
  }
  // stance 궤적이 직선이라 원호를 현으로 근사한다. 그만큼은 줄어드는 게 정상이고,
  // 그 이상 변하면 접선 계산이 틀린 것이다.
  for (let i = 0; i < 6; i++) {
    const { x: mx, z: mz, yaw } = spec.legs[i];
    const R0 = Math.hypot(mx + gr.stance * Math.cos(yaw), mz - gr.stance * Math.sin(yaw));
    const chordErr = R0 - Math.sqrt(R0 * R0 - (gr.stepLen / 2) ** 2);
    const spread = Math.max(...radii[i]) - Math.min(...radii[i]);
    assert.ok(
      spread <= chordErr * 1.15,
      `다리 ${i} 회전 시 반경이 ${(spread * 1000).toFixed(2)} mm 변한다 (현 근사 한계 ${(chordErr * 1000).toFixed(2)} mm)`
    );
  }
}

// 7. 명령이 0 이면 제자리 스텝 (수평 이동 없음)
const gz = { ...GAIT, vx: 0, vz: 0, omega: 0 };
footTargets(0.3, spec, gz, out);
for (let i = 0; i < 6; i++) {
  assert.ok(Math.abs(out[i][0] - gz.stance) < 1e-12 && Math.abs(out[i][2]) < 1e-12, `다리 ${i} 가 명령 없이 움직인다`);
}

// 7b. legFK 의 회전 방향이 three.js/Rapier 의 R_y 와 같은가.
//     IK/FK 가 자기들끼리만 일관되면 왕복 테스트는 통과하면서 로봇은 반대로 걷는다.
for (const q1 of [0.35, -0.6, 1.2]) {
  const flat = legFK(0, -0.5, -1.0, L); // q1=0 → (r, y, 0)
  const got = legFK(q1, -0.5, -1.0, L);
  const [ex, ez] = toBodyFrame(flat[0], 0, q1); // R_y(q1) 을 (r,0,0) 에 적용
  assert.ok(
    Math.hypot(got[0] - ex, got[2] - ez) < 1e-12,
    `legFK 의 q1 회전이 R_y 와 반대다 — 로봇이 반대로 걷는다 (q1=${q1})`
  );
}

// 8. 프레임 변환 왕복
for (let k = 0; k < 200; k++) {
  const x = Math.random() * 2 - 1, z = Math.random() * 2 - 1, yaw = (Math.random() - 0.5) * 7;
  const [lx, lz] = toLegLocal(x, z, yaw);
  const [bx, bz] = toBodyFrame(lx, lz, yaw);
  assert.ok(Math.hypot(bx - x, bz - z) < 1e-12, '다리 로컬 ↔ 몸통 프레임 왕복이 안 닫힌다');
}

// 9. 다리 교차 금지 — 이게 실제로 터졌던 버그다.
//    발 목표를 몸통 프레임으로 되돌린 방위가 그 다리 mount 의 방위와 같아야 한다.
//    어긋나면 다리가 몸통 아래를 가로질러 반대편으로 뻗는다.
{
  const out2 = makeTargets();
  for (const cmd of [{}, { vx: 1 }, { vx: 0, vz: 1 }, { vx: 0, omega: 1 }]) {
    const gc = { ...GAIT, ...cmd };
    for (const time of [0, 0.17, 0.4, 0.83]) {
      footTargets(time, spec, gc, out2);
      for (let i = 0; i < 6; i++) {
        const { x: mx, z: mz, yaw } = spec.legs[i];
        // 발 목표는 mount 를 원점으로 하므로, 몸통 원점 기준으로 되돌려 비교한다
        const [rx, rz] = toBodyFrame(out2[i][0], out2[i][2], yaw);
        const [fx, fz] = [mx + rx, mz + rz];
        // 발은 mount 보다 바깥에 있어야 하고, mount → 발 벡터가 다리 방향이어야 한다.
        // R_y 규약상 다리 로컬 +x 는 몸통 프레임에서 방위각 -yaw 에 해당한다.
        const outward = Math.atan2(rz, rx);
        const want = -yaw;
        const da = Math.atan2(Math.sin(outward - want), Math.cos(outward - want));
        assert.ok(Math.abs(da) < 0.6, `다리 ${i} 가 뻗는 방향에서 ${((da * 180) / Math.PI).toFixed(0)}° 벗어났다 — 교차한다`);
        assert.ok(Math.hypot(fx, fz) > Math.hypot(mx, mz), `다리 ${i} 발이 mount 안쪽으로 접혔다`);
      }
    }
  }
}

console.log('ok — IK/FK roundtrip, tripod invariant, trajectory bounds, turn tangency, no leg crossing');
