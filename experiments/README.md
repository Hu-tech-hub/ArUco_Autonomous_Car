# 개발 단계별 실험 코드

캘리브레이션·카메라·영상 인식·제어 실험을 유형별로 구분했습니다. 주행 실행 모듈은 `src`에 있습니다.

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

각 폴더는 캘리브레이션·영상 관측·주행 제어의 개발 과정을 담고 있습니다.
