# JDAMR200 프로젝트 문서

## 개요
JDAMR200은 **Arduino Mega2560과 Raspberry Pi를 활용하여 자율 이동 로봇(AMR)을 개발하는 프로젝트**입니다. 이 프로젝트는 **Arduino 기반의 모터 제어 펌웨어**와 **Python 기반의 상위 제어 소프트웨어**를 포함하고 있으며, **시리얼 통신을 통해 센서 데이터 수집 및 로봇 제어를 수행**합니다.

JDAMR200의 주요 목표:
- **자율 이동 로봇(AMR) 개발**
- **모터 제어 및 센서 데이터 수집 (MPU9250, 엔코더)**
- **Raspberry Pi와 Arduino 간의 시리얼 통신**
- **ROS2를 활용한 Odometry 계산 및 로봇 제어**

---

## 📂 디렉터리 구조
```
jdamr200/
├── arduino/                     # Arduino 펌웨어
│   ├── jdamr200_control.ino      # 메인 펌웨어 파일
│   ├── tb6612_motor.cpp          # TB6612FNG 모터 드라이버 구현
│   ├── tb6612_motor.h            # TB6612FNG 모터 드라이버 헤더 파일
│   ├── Encoder/                  # 엔코더 라이브러리
│   ├── MPU9250/                  # MPU9250 IMU 센서 라이브러리
│
├── python/                       # Python 기반 상위 제어 소프트웨어
│   ├── jdamr200_lib/              # Python 패키지 디렉토리
│   │   ├── __init__.py            # 패키지 초기화 파일
│   │   ├── jdamr200_lib.py        # 로봇 제어를 위한 핵심 코드
│   ├── setup.py                   # Python 패키지 설치 스크립트
│   ├── LIBRARY_INSTALL.md         # Python 패키지 설치 방법 설명

```

---

## 🚗 Arduino 펌웨어 개요
JDAMR200의 **Arduino 펌웨어**는 **Arduino Mega2560**에서 실행되며, **TB6612FNG 모터 드라이버와 MPU9250 IMU 센서를 활용하여 로봇의 하위 제어를 담당**합니다.

### 주요 기능
- **모터 제어**: TB6612FNG 모터 드라이버를 사용하여 4개의 바퀴를 개별적으로 제어
- **IMU 센서 데이터 수집**: MPU9250을 통해 가속도, 자이로, 자기장 데이터를 측정
- **엔코더 데이터 수집**: 각 바퀴의 이동 거리 및 속도를 측정하여 Odometry 계산
- **시리얼 통신**: Raspberry Pi로부터 모터 제어 명령을 수신하고, 센서 데이터를 전송

### 시리얼 통신 프로토콜
Arduino는 Raspberry Pi와 **바이너리 형식의 패킷**을 사용하여 통신합니다.
- **모터 제어 패킷 (Raspberry Pi → Arduino)**
  ```
  [0xF5] [0xF5] [Command] [Parameter1] [Parameter2]
  ```
- **센서 데이터 패킷 (Arduino → Raspberry Pi)**
  ```
  [0xF5] [0xF5] [IMU Data (6 bytes)] [Gyro Data (6 bytes)] [Magnetic Data (6 bytes)] [Encoders (8 bytes)] [Checksum]
  ```

### 필요한 라이브러리
1. **Encoder 라이브러리** ([다운로드](https://github.com/PaulStoffregen/Encoder))
2. **MPU9250 라이브러리** ([다운로드](https://github.com/hideakitai/MPU9250))

---

## 🏎️ Python 제어 소프트웨어 개요
JDAMR200의 **Python 기반 제어 소프트웨어**는 **Raspberry Pi에서 실행**되며, **Arduino와 시리얼 통신을 통해 로봇을 제어**합니다.

### 주요 기능
- **Arduino로 모터 제어 명령 전송**
- **IMU 및 엔코더 데이터 수신 및 파싱**
- **ROS2를 위한 Odometry 계산**
- **Thread를 사용하여 데이터 수신과 명령 전송을 병행 처리**

### Python 설치 방법
JDAMR200 Python 라이브러리는 pip를 통해 설치 가능합니다. 설치 방법은 LIBRARY_INSTALL.md 파일에 설명되어 있으며, 기본적인 설치 방법은 다음과 같습니다:
설치가 완료되면 Python 환경에서 `from jdamr200_lib import Jdamr200`를 사용하여 라이브러리를 로드할 수 있습니다.

### Python 코드 역할
JDAMR200 Python 라이브러리는 **시리얼 통신을 통해 Arduino와 데이터를 주고받으며 로봇을 제어**합니다. 
- **시리얼 포트를 통해 Arduino와 통신**
- **모터 제어 명령을 Arduino에 전송**
- **Arduino에서 송신한 IMU 및 엔코더 데이터를 수신하여 파싱**
- **받은 데이터를 바탕으로 Odometry를 업데이트**
- **Thread를 사용하여 데이터 수신과 모터 제어 명령을 동시에 수행**

---

## 🎯 결론
JDAMR200 프로젝트는 **Arduino 펌웨어와 Python 제어 소프트웨어를 결합하여 자율 이동 로봇을 제어하는 통합 시스템**입니다.
- **Arduino: 하드웨어 제어 (모터, 센서, 데이터 전송)**
- **Python: 상위 소프트웨어 제어 (ROS2, Odometry, 데이터 파싱)**

자세한 내용은 [GitHub 저장소](https://github.com/JD-edu/jdamr200)를 참고하세요! 🚀
