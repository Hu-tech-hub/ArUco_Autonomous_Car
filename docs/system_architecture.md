# 시스템 구성

## 장비

AliExpress Youfang Smart Store에서 구매한 Hiwonder Raspberry Pi AI Vision Car 보드 미포함 키트에 Raspberry Pi 5를 사용합니다. 상향 카메라로 천장을 촬영하며 메카넘 휠 구동은 Hiwonder SDK에 연결됩니다.

## 좌표와 관측

- 영상 좌표: Diamond 중심과 영상 중심의 차이로 X·Y 보정 오차 계산
- 마커 좌표: solvePnP의 rvec·tvec로 마커에 대한 상대 자세 추정
- 격자 좌표: Diamond ID 네 개를 노드 하나에 대응. 4×4 격자에서 BFS 탐색
- 방향: Rodrigues 회전 행렬에서 Yaw 계산. 다음 노드 방향과의 차이를 ±180°로 정규화

## 제어 상태

마커 미검출 → 정지

마커 검출 → 중심 보정 → 목표 노드이면 정지 → 목표 방향으로 회전 → 다음 노드 이동 → 재관측

주행 진입 코드는 src/autonomous_drive.py입니다. 캡처·제어 스레드를 분리한 개발 버전은 src/experiments/Dia_Last_2.py에 있습니다. 현재 진입 코드는 각 구동 펄스 이후 영상을 다시 읽는 순차 구조입니다.

## 설정

src/config.json에서 해상도·마커 크기·ID 매핑·모터 부호·목적지를 지정합니다. calibration_resolution은 실제 캘리브레이션 영상 크기로 입력해야 합니다. diamond_side_m은 실측 중앙 사각형의 한 변 길이로 설정합니다.

## 시각 자료

**[시스템 구성도]**

**[4×4 천장 마커 배치도]**

**[좌표계와 회전 오차 설명도]**
