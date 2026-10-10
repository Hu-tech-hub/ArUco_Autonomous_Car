# 장비 설정과 실행

## 하드웨어 연결

Raspberry Pi 5에 TurboPi 확장보드와 카메라를 연결합니다. 카메라는 천장을 촬영하도록 배치합니다. 확장보드는 I²C 버스 1의 0x7A 주소로 통신합니다.

## I²C 활성화

Raspberry Pi OS에서 I²C 인터페이스를 활성화합니다.

```bash
sudo raspi-config
# Interface Options → I2C → Enable
sudo apt install python3-venv i2c-tools
sudo usermod -aG i2c "$USER"
```

그룹 변경 후 다시 로그인합니다.

```bash
i2cdetect -y 1
# 확장보드 주소 7a 확인
```

## Python 환경

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-hardware.txt
```

프로젝트의 vendor/hiwonder 경로는 config.json에서 로딩합니다. 별도로 /home/pi/TurboPi에 파일을 복사하지 않아도 됩니다.

## 카메라·마커 설정

src/config.json에서 camera·resolution·diamond_side_m·marker_nodes를 지정합니다. yaw_sign·yaw_offset_deg·x_sign·y_sign·rotation_sign은 장착 방향에 맞춥니다. 서보 초기 위치가 필요하면 servo_initial_pulses에 채널과 펄스 값을 입력합니다.

기본 calibration_param.npz는 640×480 fisheye 파라미터입니다. 새 카메라를 사용하면 해당 카메라로 재캘리브레이션하고 생성한 JSON 경로를 설정합니다.

## 실행

```bash
# 인식·상태·로그 확인
python scripts/run_cart.py

# 실제 모터 구동
python scripts/run_cart.py --drive

# 화면 없이 실행
python scripts/run_cart.py --drive --headless
```

Esc 또는 Q로 종료하고 터미널에서는 Ctrl+C로 종료합니다. CSV는 data/logs에 저장합니다.
