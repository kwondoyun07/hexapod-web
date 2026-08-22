# assets

`phantomx.glb` — PhantomX AX Hexapod 의 시각용 메시.

원본은 [HumaRobotics/phantomx_description](https://github.com/HumaRobotics/phantomx_description)
의 STL 이고, 라이선스는 Simplified BSD (`phantomx-LICENSE.txt`, © 2013 Generation Robots).

원본 STL 13.4 MB / 195k 삼각형을 다음과 같이 가공했다.

- 각 링크를 시뮬 규약으로 정렬: **원점 = 그 링크의 조인트**, **+X = 다리가 뻗는 방향**,
  그리고 조인트 회전축을 규약 축에 맞춤. 방향만 맞추면(최단 회전) X 축 둘레가
  부품마다 제멋대로라 다리가 뒤틀린다 — 두 벡터를 같이 맞춰야 유일하게 정해진다.
- decimate 로 19k 삼각형까지 감량 → **358 KB**.
- 오브젝트 이름 `body` / `coxa` / `femur` / `tibia`. 6개 다리가 같은 메시를 공유한다
  (원본 URDF 도 좌우 구분 없이 같은 STL 을 쓴다).

물리는 이 메시를 쓰지 않는다. collider 는 언제나 primitive 이고 메시는 visual 전용이다.
