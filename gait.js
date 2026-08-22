import { legMount, toLegLocal } from './ik.js';

// Tripod gait 발끝 궤적 생성. node 로 그대로 검증한다 (test.mjs).
// 여기가 앞으로 제일 자주 만질 파일이다. 궤적 에디터를 붙인다면 shape() 만 갈아끼우면 된다.

export const GAIT = {
  // 기본값은 감이 아니라 headless 스윕으로 뽑았다. 실측: 0.15 m/s, 몸통 상하 진동 2.4 mm.
  // 로봇 형상이나 질량을 바꾸면 반드시 다시 뽑아라 — 서보 게인과 물리 주파수까지 같이 움직인다.
  height: 0.115, // 몸통 높이 (m)
  stance: 0.17, // mount 에서 발까지 수평 거리 (m). 키우면 다리가 뻗어 무릎 토크가 급증한다
  stepLen: 0.07, // 보폭 (m)
  stepHeight: 0.06, // 스윙 최고점 (m). 몸통 상하 진동보다 충분히 커야 발이 안 끌린다
  period: 1.0, // 한 주기 (s)
  duty: 0.5, // 지면 접촉 비율. 정통 tripod 가 가장 빠르고 진동도 가장 작았다
  push: 0.006, // stance 발을 지면보다 이만큼 아래로 목표한다 (m). 0 이면 발이 지면에
               // '닿아만' 있어서 수직항력이 거의 없고, 마찰을 20 까지 올려도 미끄러진다.
               // 다리가 몸을 실제로 딛게 만드는 값 — 실물에서도 같은 이유로 필요하다.
  vx: 1, // 전진 (+X 가 앞)
  vz: 0, // 게걸음
  omega: 0, // 선회. 크기는 방향 결정에만 쓴다.
            // 실측: omega > 0 이면 위에서 볼 때 반시계(yaw 증가), 8초에 약 72°.
};

export function makeTargets() {
  return Array.from({ length: 6 }, () => [0, 0, 0]);
}

/** 다리 i 의 위상 [0,1). 짝/홀 인덱스가 tripod 두 그룹. */
export function legPhase(t, i, g) {
  return (((t / g.period + (i % 2) * 0.5) % 1) + 1) % 1;
}

/**
 * 위상 → (보폭 방향 진행률 u, 들림 높이 lift).
 *
 * stance 는 발이 지면에 붙어 일정 속도로 뒤로 밀린다 (du/dph = -1/duty).
 * swing 은 단순히 앞으로 되돌리면 안 된다 — 발이 앞으로 달리는 채로 착지하면
 * 그 운동량이 마찰로 급정지하면서 몸통을 뒤로 민다. 실측으로 로봇이 실제로
 * 뒤로 걸었다. 그래서 swing 을 3차 에르미트로 잇고 양 끝 기울기를 stance 와
 * 맞춘다: 착지 순간 발은 이미 stance 속도로 뒤로 움직이고 있어 지면 대비
 * 상대속도가 0 이다. 궤적 에디터를 붙인다면 이 함수만 갈아끼우면 된다.
 */
export function shape(ph, g) {
  if (ph < g.duty) {
    return [0.5 - ph / g.duty, -(g.push || 0)];
  }
  const a = (ph - g.duty) / (1 - g.duty);
  const m = -(1 - g.duty) / g.duty; // stance 기울기를 swing 파라미터 a 기준으로 환산
  const a2 = a * a;
  const a3 = a2 * a;
  const u =
    (2 * a3 - 3 * a2 + 1) * -0.5 + // h00 * p0
    (a3 - 2 * a2 + a) * m + //        h10 * m0
    (-2 * a3 + 3 * a2) * 0.5 + //     h01 * p1
    (a3 - a2) * m; //                 h11 * m1
  return [u, g.stepHeight * Math.sin(Math.PI * a)];
}

/** 6개 다리의 발 목표를 각자의 로컬 프레임으로 채운다. */
export function footTargets(t, spec, g, out) {
  for (let i = 0; i < 6; i++) {
    const yaw = spec.legYaw[i];
    const [mx, , mz] = legMount(yaw, spec.mountR);

    // 몸통 프레임 진행 방향 = 병진 + 회전 접선속도 (ω × r)
    let sx = g.vx + g.omega * mz;
    let sz = g.vz - g.omega * mx;
    const n = Math.hypot(sx, sz);
    if (n > 1e-6) {
      sx = (sx / n) * g.stepLen;
      sz = (sz / n) * g.stepLen;
    } else {
      sx = sz = 0; // 명령 없음 → 제자리 스텝
    }

    const [lx, lz] = toLegLocal(sx, sz, yaw);

    const [u, lift] = shape(legPhase(t, i, g), g);
    out[i][0] = g.stance + lx * u;
    out[i][1] = -g.height + lift;
    out[i][2] = lz * u;
  }
  return out;
}
