# 사진·영상 자료

| 파일 | 내용 |
| --- | --- |
| drive.mp4 | 실제 주행 영상 · 약 36초 |
| recognition_log.mp4 | 실제 마커 인식·로그 영상 · 약 36초 |
| drive_video_preview.jpg | 주행 영상에서 추출한 썸네일 |
| log_video_preview.jpg | 로그 영상에서 추출한 썸네일 |
| cart_complete.png | 완성 카트 사진 |
| drive_start.png · drive_arrival.png | 출발·도착 장면 |
| diamond_before.png · diamond_detected.png | Diamond 검출 전·후 |
| ceiling_layout.png | 천장 4×4 배치 개념도 |
| navigation_flow.png | 주행 피드백 흐름도 |
| pose_control_concept.png | 위치·방향·회전 오차 개념도 |
| vision_control_overview.png | Calibration·Detection·Pose·PID 설명 개념도 |
| feature_localization_concept.png | 특징점·Homography·Motion Blur 설명 개념도 |

설명 개념도는 구현 원리를 시각화한 자료입니다. 주행과 인식 화면은 실제 영상에서 확인할 수 있습니다.

## 원자료에서 추가한 기술 설명 이미지

- `feature_matching_reference.png` · `matching_reference_alt.png`: 특징점 매칭 설명 예시
- `homography_reference.png` · `homography_reference_alt.png`: Homography 영역 추정 설명 예시
- `calibration_board_types.png`: Chessboard·ArUco·ChArUco 비교
- `diamond_example_before.png` · `diamond_example_detected.png`: Diamond 검출·좌표축 설명 예시
- `camera_projection_model.png`: 카메라 투영 모델
- `expansion_board_interfaces.png`: 확장보드 인터페이스

출처 문서: `[캡디]김상엽 진행 내용 보고.docx`와 `캡스톤 디자인 결과 보고서.docx`에 수록된 이미지.

## 동시 재생 GIF

- `drive.gif`: 주행 영상 GIF
- `recognition_log.gif`: 마커 인식·로그 GIF
- 동일한 480×360 크기와 초당 8프레임으로 전체 영상을 반복 재생합니다.
