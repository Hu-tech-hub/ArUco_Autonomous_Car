# 코드 분석과 기능별 정리

## 파일 분류

전달받은 Python 코드 15개를 분석해 캘리브레이션·카메라·영상 인식·제어로 나눴습니다. 원본은 experiments에 보존하며 현행 실행 모듈은 src에 있습니다.

| 분류 | 원본 | 확인한 역할 |
| --- | --- | --- |
| calibration | Calibration_2·Calibration_3 | 보드 캘리브레이션과 저장 파라미터 적용 |
| camera | Camera | 캡처 스레드와 fisheye remap |
| vision | Centering·Centering_2·Last | 영상 중심·공간 위치·방향 표시 |
| vision | Dia_Detect·Dia_Detect_1 | Diamond 선택과 solvePnP |
| control | Last_move·Last_move_ss·Dia_Last_1 | 보드 기반 중심/회전 보정 |
| control | Dia_Last_2 | Diamond·ID 매핑·4×4 BFS·PID 통합 |
| control | EmptyFunc·real_last | 추종 개발 코드. real_last는 콜론 누락으로 문법 오류 |
| control | Rotate | LAB 색상 마스크·형태학 연산·윤곽선 추종 |

## 문제와 구현 변경

| 원본 문제 | 현재 처리 |
| --- | --- |
| 선택 ID와 마지막 마커 중심이 섞임 | Diamond 객체에 ID·노드·코너를 함께 보관 |
| 중심 판정이 한 축만 만족해도 완료 | 양 축 모두 허용 범위 안인지 확인 |
| 이전 ID 대입 방향과 이동 상태 불명확 | current_node·next_node·departed_at로 상태 관리 |
| 목적지 angle=None과 경로 불일치 혼동 | 목적지 중심·연속 안정화 조건 후 ARRIVED |
| 전역 관측을 여러 스레드가 공유 | 카메라만 스레드로 분리. 제어는 한 스레드의 관측 객체 사용 |
| sleep 시간과 영상 주기가 혼동될 수 있음 | 캡처 시각·새 프레임 조건·실제 처리 시간 CSV 기록 |
| 고정 회전 명령과 PID 크기 연결 부족 | 제한된 PID 출력을 각속도 명령으로 전달 |
| 파라미터 키·경로가 파일마다 다름 | K/D·K/dists·k_array/d_array 형식을 구분해 로딩 |
| fisheye와 일반 왜곡 모델 혼재 | fisheye 코너 보정 후 PnP. 원래 모델로 재투영 검증 |
| 캘리브레이션 imageSize 순서 오류 가능 | 너비·높이 순으로 저장 |
| 미검출·종료 처리가 버전마다 다름 | 관측 실패·오래된 관측·카메라 오류·타임아웃·종료 시 정지 |

## 검증

OpenCV 4.11에서 10개 테스트 통과. 실제 생성 마커의 검출과 합성 pinhole·fisheye 위치 복원 및 전체 파이프라인의 목적지 정지·CSV 기록을 검증했습니다. 원본 미완성 파일은 테스트 대상 실행 모듈이 아닙니다.

## 장비에서 맞출 값

마커 실측 크기·카메라 설치 방향·노드 ID 순서·천장 마커의 방향·모터 부호·PID 게인·모터 명령 단위·속도·허용 오차입니다. 현재 소프트웨어 테스트 결과를 실제 주행 성능으로 해석하지 않습니다.
