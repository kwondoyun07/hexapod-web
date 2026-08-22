import * as THREE from 'three';
import * as RAPIER from '@dimforge/rapier3d-compat';
import { legIK, legMount } from './ik.js';

export { legIK, legFK } from './ik.js';

// 로봇 정의. 물리는 항상 이 primitive 로 돈다.
// Blender 메시는 나중에 visual 로만 덮는다 — collider 를 메시로 바꾸지 마라, 느리고 불안정하다.
export const SPEC = {
  body: { r: 0.115, h: 0.045 },
  leg: { coxa: 0.055, femur: 0.1, tibia: 0.145, thick: 0.013, footR: 0.016 },
  mountR: 0.1,
  // 실측 질량 (kg). density 로 잡으면 안 된다 — 링크 부피는 작은데 실제로는
  // 링크마다 서보가 통째로 붙어서 훨씬 무겁다. 너무 가벼우면 관성이 작아
  // 서보 오버슈트가 그대로 떨림이 되고, 몸통:링크 질량비가 커져 솔버도 나빠진다.
  // PhantomX 급 기준. 저울에 올려보고 여기부터 맞춰라.
  mass: { body: 1.2, coxa: 0.075, femur: 0.07, tibia: 0.055, foot: 0.008 },
  // [앞우, 중우, 뒤우, 뒤좌, 중좌, 앞좌] — 짝/홀 인덱스가 그대로 tripod 그룹이 된다
  legYaw: [45, 90, 135, -135, -90, -45].map((d) => (d * Math.PI) / 180),
};

// 로봇 링크끼리는 서로 통과, 지면과만 충돌한다 (self-collision 은 조인트 안정성만 해친다)
const G_ROBOT = 0x00010002;
const G_GROUND = 0x00020001;

const AX_Y = new THREE.Vector3(0, 1, 0);
const AX_Z = new THREE.Vector3(0, 0, 1);

// 리셋 때마다 world 를 새로 만드므로 collider 만 만든다. 씬 장식은 main 이 한 번만 올린다.
export function buildGround(world, friction) {
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(25, 0.1, 25)
      .setTranslation(0, -0.1, 0)
      .setFriction(friction)
      .setCollisionGroups(G_GROUND)
  );
}

/**
 * 헥사포드 조립. 중립 자세(stand)를 FK 로 그대로 배치하므로 시작하자마자 튀지 않는다.
 * @param stand {height, reach} — 몸통 높이, mount 에서 발까지 수평 거리
 */
export function buildHexapod(world, scene, stand) {
  const { leg: L, body: B, legYaw, mountR, mass } = SPEC;
  // height 는 몸통 중심 → 발 '중심' 거리다. 발 ball 반지름을 안 더하면
  // 시작부터 발 6개가 지면을 footR 만큼 뚫고 들어가 로봇이 튀어오른다.
  const bodyY = stand.height + L.footR + 0.001;
  const disposables = [];

  const bodyRb = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0, bodyY, 0));
  world.createCollider(
    RAPIER.ColliderDesc.cylinder(B.h / 2, B.r).setMass(mass.body).setCollisionGroups(G_ROBOT),
    bodyRb
  );

  const meshes = [];
  const bodyMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(B.r, B.r, B.h, 6),
    new THREE.MeshStandardMaterial({ color: 0x40606f, metalness: 0.35, roughness: 0.55 })
  );
  bodyMesh.castShadow = true;
  scene.add(bodyMesh);
  meshes.push([bodyRb, bodyMesh]);
  disposables.push(bodyMesh);

  // 전진(+X) 방향 표시. 어느 쪽이 앞인지 모르면 gait 부호를 영원히 헷갈린다.
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.022, 0.06, 8),
    new THREE.MeshStandardMaterial({ color: 0x9bff6a })
  );
  nose.rotation.z = -Math.PI / 2;
  nose.position.set(B.r + 0.02, 0, 0);
  bodyMesh.add(nose);

  const joints = [];
  const q0 = legIK(stand.reach, -stand.height, 0, L);
  const lens = [L.coxa, L.femur, L.tibia];

  for (let i = 0; i < 6; i++) {
    const yaw = legYaw[i];
    const mount = new THREE.Vector3(...legMount(yaw, mountR));
    const color = i % 2 === 0 ? 0x2ec4b6 : 0xff8a3d; // tripod 그룹 A / B

    const rot = new THREE.Quaternion().setFromAxisAngle(AX_Y, yaw + q0[0]);
    let tip = mount.clone();
    const parts = [];

    for (let s = 0; s < 3; s++) {
      if (s > 0) rot.multiply(new THREE.Quaternion().setFromAxisAngle(AX_Z, q0[s]));
      const half = new THREE.Vector3(lens[s] / 2, 0, 0).applyQuaternion(rot);
      const center = tip.clone().add(half);
      tip = center.clone().add(half);

      const rb = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic()
          .setTranslation(center.x, center.y + bodyY, center.z)
          .setRotation({ x: rot.x, y: rot.y, z: rot.z, w: rot.w })
      );
      world.createCollider(
        RAPIER.ColliderDesc.cuboid(lens[s] / 2, L.thick, L.thick)
          .setMass([mass.coxa, mass.femur, mass.tibia][s])
          .setFriction(0.8)
          .setCollisionGroups(G_ROBOT),
        rb
      );

      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(lens[s], L.thick * 2, L.thick * 2),
        new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.7 })
      );
      mesh.castShadow = true;
      scene.add(mesh);
      meshes.push([rb, mesh]);
      disposables.push(mesh);
      parts.push(rb);

      if (s === 2) {
        // 발끝만 별도 ball. 박스 모서리로 지면을 긁으면 접촉이 지저분해진다.
        world.createCollider(
          RAPIER.ColliderDesc.ball(L.footR)
            .setTranslation(lens[2] / 2, 0, 0)
            .setMass(mass.foot)
            .setFriction(1.6)
            .setCollisionGroups(G_ROBOT),
          rb
        );
        const foot = new THREE.Mesh(
          new THREE.SphereGeometry(L.footR, 14, 10),
          new THREE.MeshStandardMaterial({ color: 0xe8eef0, roughness: 0.9 })
        );
        foot.position.set(lens[2] / 2, 0, 0);
        mesh.add(foot); // tibia 메시의 자식 = 동기화 공짜
      }
    }

    // revolute anchor 는 각 바디의 로컬 좌표, axis 는 두 바디 공통 로컬 축
    const anchors = [
      [{ x: mount.x, y: 0, z: mount.z }, { x: -L.coxa / 2, y: 0, z: 0 }, { x: 0, y: 1, z: 0 }],
      [{ x: L.coxa / 2, y: 0, z: 0 }, { x: -L.femur / 2, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }],
      [{ x: L.femur / 2, y: 0, z: 0 }, { x: -L.tibia / 2, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }],
    ];
    const chain = [bodyRb, ...parts];
    const legJoints = [];
    for (let s = 0; s < 3; s++) {
      const jd = RAPIER.JointData.revolute(anchors[s][0], anchors[s][1], anchors[s][2]);
      const j = world.createImpulseJoint(jd, chain[s], chain[s + 1], true);
      j.setContactsEnabled(false);
      // ForceBased 라야 stiffness 가 N·m/rad, maxForce 가 N·m 로 직접 읽힌다.
      // AccelerationBased 는 게인이 링크 관성으로 나눠져 서보 스펙과 대응이 안 된다.
      j.configureMotorModel(RAPIER.MotorModel.ForceBased);
      legJoints.push(j);
    }
    joints.push(legJoints);
  }

  function sync() {
    for (const [rb, mesh] of meshes) {
      const p = rb.translation();
      const r = rb.rotation();
      mesh.position.set(p.x, p.y, p.z);
      mesh.quaternion.set(r.x, r.y, r.z, r.w);
    }
  }

  function dispose() {
    for (const m of disposables) {
      m.removeFromParent();
      m.geometry.dispose();
      m.material.dispose();
    }
  }

  /**
   * 조인트 목표각 적용.
   * coxa 는 mount yaw 가 초기 조립에 이미 들어가 있으므로 target 에 yaw 를 더해준다.
   */
  function drive(targets, servo) {
    for (let i = 0; i < 6; i++) {
      const q = legIK(targets[i][0], targets[i][1], targets[i][2], L);
      const j = joints[i];
      j[0].configureMotorPosition(legYaw[i] + q[0], servo.stiffness, servo.damping);
      j[1].configureMotorPosition(q[1], servo.stiffness, servo.damping);
      j[2].configureMotorPosition(q[2], servo.stiffness, servo.damping);
      for (let s = 0; s < 3; s++) j[s].setMotorMaxForce(servo.maxForce);
    }
  }

  return { bodyRb, joints, drive, sync, dispose };
}
