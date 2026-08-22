// 아날로그 스틱 하나. 키보드와 포인터 드래그 둘 다 받는다.
// 값은 x, y ∈ [-1, 1] (화면 기준: y 는 아래가 +). 원 밖으로는 안 나간다.

const FOLLOW = 0.15; // 목표를 향해 프레임당 따라가는 비율

export function makeStick(el, keymap, label) {
  el.classList.add('stick');
  el.innerHTML = `<div class="knob"></div><span class="cap">${label}</span>`;
  const knob = el.querySelector('.knob');

  const st = { x: 0, y: 0, tx: 0, ty: 0, dragging: false };

  const fromPointer = (e) => {
    const r = el.getBoundingClientRect();
    let dx = (e.clientX - r.left - r.width / 2) / (r.width / 2);
    let dy = (e.clientY - r.top - r.height / 2) / (r.height / 2);
    const m = Math.hypot(dx, dy);
    if (m > 1) {
      dx /= m;
      dy /= m;
    }
    st.tx = dx;
    st.ty = dy;
  };
  const release = () => {
    st.dragging = false;
    st.tx = st.ty = 0;
  };

  el.addEventListener('pointerdown', (e) => {
    st.dragging = true;
    el.setPointerCapture(e.pointerId);
    fromPointer(e);
  });
  el.addEventListener('pointermove', (e) => st.dragging && fromPointer(e));
  el.addEventListener('pointerup', release);
  el.addEventListener('pointercancel', release);

  /** @param pressed 눌린 KeyboardEvent.code 의 Set */
  st.update = (pressed) => {
    if (!st.dragging) {
      let tx = (pressed.has(keymap.right) ? 1 : 0) - (pressed.has(keymap.left) ? 1 : 0);
      let ty = (pressed.has(keymap.down) ? 1 : 0) - (pressed.has(keymap.up) ? 1 : 0);
      const m = Math.hypot(tx, ty);
      if (m > 1) {
        tx /= m; // 대각선이 더 빨라지지 않게
        ty /= m;
      }
      st.tx = tx;
      st.ty = ty;
    }
    // 명령을 그대로 꽂으면 로봇이 넘어진다. 목표를 향해 부드럽게 따라간다.
    st.x += (st.tx - st.x) * FOLLOW;
    st.y += (st.ty - st.y) * FOLLOW;
    knob.style.transform = `translate(${st.x * 34}%, ${st.y * 34}%)`;
    el.classList.toggle('active', Math.hypot(st.x, st.y) > 0.02);
    return st;
  };

  return st;
}

/** 스틱들이 쓰는 키를 모아 준다 (스크롤 같은 기본 동작을 막을 때 쓴다). */
export function keysOf(...keymaps) {
  return new Set(keymaps.flatMap((k) => Object.values(k)));
}
