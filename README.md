# README.md 작성 (가장 중요한 문서!)
README.md는 프로젝트의 핵심 소개 문서이므로, 누가 봐도 이해할 수 있도록 깔끔하게 작성해야 함.

## 포함할 내용

### (1) 프로젝트 개요

ArUco Marker 기반 자율주행 자동차 🚗💨

📌 프로젝트 개요
ArUco Marker를 활용하여 경로를 인식하고, 자율주행하는 자동차를 개발하는 프로젝트입니다.
Python과 OpenCV를 사용하여 마커를 인식하고, 모터를 제어하는 방식으로 동작합니다.

### (2) 시스템 구조

✅ 다이어그램 포함하면 좋음

⚙️ 시스템 구조
1. 카메라가 ArUco 마커를 감지
2. OpenCV를 활용해 마커의 위치 계산
3. 경로를 결정하고 모터 제어


### (3) 실행 방법

✅ 필수 라이브러리 설치

🚀 실행 방법

1️⃣ 필수 라이브러리 설치

```bash
pip install -r requirements.txt
```

2️⃣ ArUco 마커 인식 실행
```bash
python src/detect_aruco.py
```

3️⃣ 자율주행 실행

```bash
python src/autonomous_drive.py
```

### 📌 (4) 실험 결과  

✅ 스크린샷 & 테스트 영상 포함 

```md
📊 실험 결과
- ArUco 마커 인식 결과  
![ArUco Detection](docs/aruco_detection_example.png)

- 테스트 주행 영상  
[![Watch the video](docs/test_video_thumbnail.png)](https://youtu.be/sample_video_link)

### 📌 (5) 추가 개선 방향

```md
🔄 추가 개선 방향
- YOLO 기반 ArUco 탐지 성능 개선
- PID 제어 적용하여 주행 안정성 향상
```