# 캘리브레이션과 실험 기록

| 파일 | 형식 | 영상 크기 | 사용 |
| --- | --- | --- | --- |
| calibration_param.npz | fisheye · k_array/d_array/dim_array | 640×480 | 현재 기본 설정 |
| calib_1109.npz | K/dists | 미기록 | 사용 시 calibration_resolution 입력 |
| calibration.json | calib_1109.npz를 일반 K/D JSON으로 변환 | 미기록 | 대체 설정 또는 재캘리브레이션 출력 |

scripts/calibrate_camera.py는 일반 pinhole ChArUco 캘리브레이션을 수행합니다. 실제 인쇄한 보드의 길이를 입력해야 합니다. 출력 JSON에는 image_size와 RMS 재투영 오차를 함께 저장합니다.

주행 CSV는 data/logs에 생성됩니다. 사용자 장비에서 실행한 로그는 자동으로 Git에 포함되지 않습니다.

**[실험 결과 CSV]**

**[처리 시간 비교 표]**
