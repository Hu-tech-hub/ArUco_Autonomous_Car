# 코드 분석

## 분석 범위

전달받은 Python 파일 15개의 함수·호출·파라미터 로딩·제어 상태를 정적으로 검토했습니다. 보드와 카메라를 연결한 실행 검증은 하지 않았습니다. 파일명만으로 최종 버전을 판단하지 않습니다.

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| Calibration_2.py | ChArUco 관측 수집·내부 파라미터 계산·보드 자세 표시 |
| Calibration_3.py | Camera 모듈의 저장 파라미터로 보드 자세 표시 |
| Camera.py | 별도 캡처 스레드·fisheye 보정 맵 생성·remap |
| Centering.py | ChArUco 보드 기준 위치·중심 관측 실험 |
| Centering_2.py | 영상 중심과 투영 좌표의 거리·방향 표시 |
| Last.py | 저장 파라미터 로딩 또는 재캘리브레이션·위치 표시 |
| Dia_Detect.py | 가까운 Diamond 선택·solvePnP·축 표시 |
| Dia_Detect_1.py | Diamond 자세 추정 기본 실험 |
| EmptyFunc.py | 보드 관측을 서보·차량 추종 제어에 연결한 실험 |
| Last_move_ss.py | Yaw 오차를 이용한 회전 제어 실험 |
| Last_move.py | 중심 보정·회전 PID·캡처 큐를 결합한 보드 기반 코드 |
| Dia_Last_1.py | 명칭과 달리 run은 ChArUco 보드 자세 추정. Diamond 통합본으로 볼 수 없음 |
| Dia_Last_2.py | Diamond 인식·ID-노드 매핑·4×4 BFS·중심/회전 제어 통합 |
| real_last.py | 미완성 추종 코드. 271행 if 문에 콜론 누락으로 파싱 실패 |
| Rotate.py | Gaussian Blur·LAB 색상 마스크·형태학 처리·윤곽선 기반 색상 추종 |

## Dia_Last_2.py 핵심 문제

1. find_closest_marker_to_center는 가장 가까운 ID를 반환하지만 전역 중심 좌표는 루프 마지막 마커의 좌표로 남습니다. 선택 ID와 좌표가 달라질 수 있습니다.
2. 중심 완료를 X 또는 Y 한 축만 허용 범위에 들어오면 참으로 판정합니다. 양 축 조건이 필요합니다.
3. 이동 완료 뒤 closest_id = prev_id로 대입합니다. 이전 관측을 저장하는 목적이라면 방향이 반대입니다.
4. 목적지 노드의 angle=None과 경로에 없는 노드의 None을 구분하지 않습니다. 목적지 종료 상태가 명확하지 않습니다.
5. 입력 감시 스레드가 새 input을 읽지 않고 시작 때 입력한 user_input만 반복 확인합니다.
6. move 스레드만 Lock을 획득하며 관측을 갱신하는 run은 동일 Lock을 사용하지 않습니다. 여러 전역값을 일관된 관측으로 읽는 것을 보장하지 않습니다.
7. PID 출력을 연속 모터 속도 대신 구동 시간으로 변환합니다. 제어 방식은 영상 재관측을 사이에 둔 펄스 제어입니다. sleep 시간만으로 실제 영상 갱신 주기를 주장할 수 없습니다.

## 캘리브레이션과 좌표

- calibration_param.npz: dim_array·k_array·d_array 저장. Camera.py는 cv2.fisheye로 사용합니다.
- calib_1109.npz: K·dists 저장. 현재 data/calibration.json은 이 파일을 변환했습니다.
- 여러 원본은 존재하지 않는 K·D 키로 calibration_param.npz를 읽습니다. 그대로는 KeyError가 발생합니다.
- fisheye 계수와 일반 pinhole 계수를 같은 solvePnP 인자로 무조건 교환하면 안 됩니다.
- Calibration_2.py는 gray.shape를 imsize로 전달해 높이·너비 순서입니다. OpenCV imageSize의 너비·높이 순서와 대조해야 합니다.
- 보정 영상 크기와 실제 입력 크기를 맞춰야 합니다. 원본의 size=(640,480)은 실제 카메라 해상도를 강제하지 않습니다.
- Diamond object_points의 코너 순서와 실제 반환 코너의 대응 및 카메라 설치 방향은 장비에서 확인해야 합니다.
- markerLength를 공간 사각형 크기로 사용하는 원본은 실제 중앙 Diamond 길이와 일치하는지 확인해야 합니다.

## 현재 진입 코드와 원본의 차이

src/autonomous_drive.py는 선택한 Diamond의 중심과 ID를 함께 쓰고 양 축 중심 조건·각도 오차 정규화·목적지 정지·관측 실패 시 정지를 처리합니다. 주행 로직을 하나의 파일에 정리한 상태이며 기능별 모듈 분리는 아직 하지 않았습니다.

현재 진입 코드는 순차 캡처와 구동 펄스 구조입니다. 원본의 캡처·제어 멀티스레딩이 그대로 이전된 것은 아닙니다. README의 멀티스레딩 설명은 원본 개발 코드에 관한 설명입니다.

모터 회전은 PID 출력의 부호로 방향을 결정하되 속도와 펄스 길이는 설정값으로 고정됩니다. 따라서 PID 출력 크기에 비례한 회전량 제어라고 표현하면 안 됩니다. 다음 구현 정리에서 이 부분을 포함해 시간 동작과 좌표 부호를 검증해야 합니다.

## 다음 구조 정리

camera / calibration / marker_detection / pose_estimation / path_planning / control / autonomous_drive로 책임을 나누고 실험 코드는 별도로 보존하는 것이 적절합니다. 현재 experiments 디렉터리는 실행 가능한 공통 모듈 모음이 아니라 개발 기록입니다.
