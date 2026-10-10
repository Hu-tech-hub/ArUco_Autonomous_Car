# Hiwonder TurboPi 하드웨어 지원

Hiwonder TurboPi의 메카넘 섀시 SDK와 확장보드 통신 인터페이스입니다.

| 파일 | 기능 |
| --- | --- |
| HiwonderSDK/mecanum.py | Hiwonder 메카넘 휠 속도 계산 |
| HiwonderSDK/Board.py | 프로젝트의 I²C 모터·서보 인터페이스 연결 |
| HiwonderSDK/PID.py | Hiwonder PID 유틸리티 |
| HiwonderSDK/Misc.py | Hiwonder 좌표·범위 변환 유틸리티 |
| reference/Board.py | Hiwonder 확장보드 SDK |
| reference/yaml_handle.py | Hiwonder YAML 설정 로딩 |
| reference/servo_config.yaml | 기본 서보 설정 |

출처: Hiwonder TurboPi RPi 4B Version Source Code.

메카넘 휠 계산은 Hiwonder SDK를 사용하고 모터 통신은 src/hardware.py의 ExpansionBoard에 연결했습니다. 모터·서보 경로는 I²C로 분리해 GPIO와 RGB LED 초기화 없이 동작하도록 구성했습니다.

확장보드 주소는 0x7A이며 모터 레지스터는 31~34입니다. setMotor는 −100~100 명령을 사용합니다. 제조사 극성 규칙에 따라 모터 1·3의 부호를 반전합니다.

LED·초음파·라인 센서 데모는 마커 주행 파이프라인에 사용하지 않습니다.

[Hiwonder 공식 제품](https://www.hiwonder.com/products/turbopi)

[TurboPi 공식 문서](https://docs.hiwonder.com/projects/TurboPi/en/standard/)
