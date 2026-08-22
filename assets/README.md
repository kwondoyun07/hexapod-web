# assets

`phantomx.glb` — PhantomX AX Hexapod 의 시각용 메시.

원본은 [HumaRobotics/phantomx_description](https://github.com/HumaRobotics/phantomx_description)
의 STL 이고, 라이선스는 Simplified BSD (`phantomx-LICENSE.txt`, © 2013 Generation Robots).

원본 STL 13.4 MB / 195k 삼각형을 다음과 같이 가공했다.

- 각 링크를 시뮬 규약으로 정렬: **원점 = 그 링크의 조인트**, **+X = 다리가 뻗는 방향**,
  그리고 조인트 회전축을 규약 축에 맞춤. 방향만 맞추면(최단 회전) X 축 둘레가
  부품마다 제멋대로라 다리가 뒤틀린다 — 두 벡터를 같이 맞춰야 유일하게 정해진다.
- **coxa 는 부품 두 개를 합친 것이다.** URDF 의 `c1`/`c2` 는 고정 조인트로 묶인 별개
  링크이고 둘 다 `connect.STL` 을 쓴다 — 실물에서 2번 모터를 감싸는 브래킷 한 쌍이다.
  하나만 넣으면 그 사출물이 통째로 없다.
- DISSOLVE(평면 병합)로 55k 삼각형까지 감량. 비율 기반 COLLAPSE 로 깎으면 이 부품들의
  평면과 격자 구멍이 먼저 뭉개진다. 각도는 부품마다 다르다 — 곡면이 많은 tibia 는
  4°, 평면 위주인 body 는 6°.
- 각도(35°) 기반 셰이딩. 전부 smooth 로 칠하면 각진 기계 부품이 뭉개져 보인다.
  대신 정점마다 노멀이 갈라져 파일이 4.5 MB 로 불어나므로 **Draco 로 압축 → 576 KB**.
  (읽는 쪽은 `main.js` 가 DRACOLoader 를 붙인다.)
- 오브젝트 이름 `body` / `coxa` / `femur` / `tibia`. 6개 다리가 같은 메시를 공유한다
  (원본 URDF 도 좌우 구분 없이 같은 STL 을 쓴다).

물리는 이 메시를 쓰지 않는다. collider 는 언제나 primitive 이고 메시는 visual 전용이다.
