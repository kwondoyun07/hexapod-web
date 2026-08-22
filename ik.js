// 다리 기구학. 의존성 없음 — node 로 그대로 돌려 검증할 수 있다 (test.mjs).
// 프레임: 다리 로컬, 원점 = coxa 조인트, +X = 다리가 뻗는 방향, -Y = 아래.
//
// q1 의 부호는 three.js / Rapier 의 R_y 규약을 따른다: +q1 은 -Z 로 회전한다.
// 수학 표준(atan2(z,x), +q1 이 +Z)을 쓰면 IK/FK 왕복은 멀쩡히 통과하면서 —
// 둘 다 같은 규약이니까 — 실제 로봇만 좌우가 뒤집혀 반대로 걷는다. 실제로 그랬다.
// 정지 자세는 로컬 z=0 이라 증상이 안 나타나고 걸을 때만 드러난다.

/** 발 목표 → 조인트 3각. 닿을 수 없는 목표는 "최대한 뻗은 자세"로 clamp 된다. */
export function legIK(px, py, pz, L) {
  const q1 = Math.atan2(-pz, px);
  const r = Math.hypot(px, pz) - L.coxa;
  const h = py;
  const f = L.femur;
  const t = L.tibia;
  const c3 = (r * r + h * h - f * f - t * t) / (2 * f * t);
  const q3 = -Math.acos(Math.min(1, Math.max(-1, c3))); // 무릎은 아래로 굽는다
  const q2 = Math.atan2(h, r) - Math.atan2(t * Math.sin(q3), f + t * Math.cos(q3));
  return [q1, q2, q3];
}

/** 조인트 3각 → 발 위치. IK 검증용이자 발 궤적 시각화용. */
export function legFK(q1, q2, q3, L) {
  const r = L.coxa + L.femur * Math.cos(q2) + L.tibia * Math.cos(q2 + q3);
  const y = L.femur * Math.sin(q2) + L.tibia * Math.sin(q2 + q3);
  return [r * Math.cos(q1), y, -r * Math.sin(q1)];
}

// --- 다리 배치 변환 ---------------------------------------------------
// three.js 의 R_y(θ) 는 (1,0,0) → (cosθ, 0, -sinθ) 다. z 부호를 손으로 쓰다 한 번
// 틀리면 다리가 몸통을 가로질러 반대편으로 뻗는다. 그래서 규칙을 여기 한 곳에만 둔다.
// mount 위치는 SPEC.legs 가 실측값으로 들고 있다 — yaw 에서 계산하지 않는다.

/** 몸통 프레임 벡터 → 다리 로컬 (R_y(-yaw)) */
export function toLegLocal(x, z, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [x * c - z * s, x * s + z * c];
}

/** 다리 로컬 벡터 → 몸통 프레임 (R_y(yaw)) */
export function toBodyFrame(x, z, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [x * c + z * s, -x * s + z * c];
}
