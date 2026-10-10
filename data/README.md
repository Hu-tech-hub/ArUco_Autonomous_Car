# 캘리브레이션·실험 데이터

calibration.json은 제공된 calib_1109.npz의 K·dists를 변환한 파일입니다. 원래 캘리브레이션 영상 크기는 파일에 없으므로 확인 후 src/config.json에 입력해야 합니다.

scripts/calibrate_camera.py로 새 K·D와 영상 크기 및 RMS 재투영 오차를 저장할 수 있습니다.

[실험 로그 CSV]

[처리 시간 측정 결과]
