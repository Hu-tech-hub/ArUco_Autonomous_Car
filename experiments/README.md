# 개발 단계별 원본 코드

원본 파일은 유형별로 보존했습니다. 현행 실행 코드는 ../src에 있습니다. 이 폴더 전체를 공통 실행 모듈로 import하지 않습니다.

| 폴더 | 파일 | 내용 |
| --- | --- | --- |
| calibration | Calibration_2·Calibration_3 | ChArUco 캘리브레이션·파라미터 적용 실험 |
| camera | Camera | 카메라 스레드·fisheye remap |
| vision | Centering·Centering_2·Last | 중심·위치·방향 표시 실험 |
| vision | Dia_Detect·Dia_Detect_1 | Diamond 검출·PnP |
| control | Last_move·Last_move_ss·Dia_Last_1 | 보드 기반 이동·회전 실험 |
| control | Dia_Last_2 | Diamond·BFS·PID 통합 개발본 |
| control | EmptyFunc·real_last | 추종 제어 개발 코드 |
| control | Rotate | 이름과 달리 LAB 색상 추종 코드 |

real_last.py에는 문법 오류가 있습니다. 원본 상태를 보존했으며 실행 진입점으로 사용하지 않습니다. 다른 파일도 장비 SDK·경로·파라미터 키와 OpenCV API 버전이 서로 다를 수 있습니다.
