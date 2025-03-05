# JDAMR200 Python 라이브러리 문서

## 개요
JDAMR200 Python 라이브러리는 **Arduino Mega2560 기반의 AMR(Autonomous Mobile Robot)**을 **Raspberry Pi에서 제어**하기 위해 개발되었습니다.
이 라이브러리는 다음과 같은 주요 기능을 제공합니다:

- **Arduino와 시리얼 통신을 통해 모터 제어**
- **IMU(MPU9250) 및 엔코더 데이터를 수신 및 파싱**
- **ROS2를 위한 Odometry 계산**
- **Thread를 사용하여 데이터 수신과 명령 전송을 병행 처리**

---

## 📂 디렉터리 구조(주요 파일만 표시)
```
python/jdamr200_lib
├── jdamr200_lib.py         # 핵심 Python 라이브러리
│   ├── _init_.py           #
│   ├── jdamr200_lib.py     # 로봇 제어 파이썬 코드 
├── setup.py                # Python 패키지 설치 스크립트
├── LIBRARY_INSTALL.md      # Python 라이브러리 설치 방법 설명
```

---

## 📦 Python 패키지 설치
JDAMR200 Python 라이브러리는 **pip를 통해 설치 가능**합니다.
설치 방법은 `LIBRARY_INSTALL.md` 파일에 설명되어 있으며, 기본적인 설치 방법은 다음과 같습니다:

설치가 완료되면 Python 환경에서 `from jdamr200_lib import Jdamr200 `를 사용하여 라이브러리를 로드할 수 있습니다.

---

## 📡 Arduino ↔ Raspberry Pi 시리얼 통신
JDAMR200 Python 라이브러리는 **Arduino로 모터 제어 명령을 보내고, IMU 및 엔코더 데이터를 수신**합니다.

### 데이터 송수신 개요
- **모터 제어 패킷 (Raspberry Pi → Arduino)**
  ```
  [0xF5] [0xF5] [Command] [Parameter1] [Parameter2]
  ```
- **센서 데이터 패킷 (Arduino → Raspberry Pi)**
  ```
  [0xF5] [0xF5] [Accel Data (6 bytes)] [Gyro Data (6 bytes)] [Magnetic Data (6 bytes)] [Encoders (8 bytes)] [Checksum]
  ```

---

## 🏎️ JDAMR200 핵심 Python 코드 (`jdamr200_lib.py`)
JDAMR200 Python 라이브러리의 핵심 기능을 설명합니다.

### 1️⃣ **클래스 초기화**
```python
class Jdamr200(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
```
- **시리얼 포트 설정 (`/dev/ttyACM0`)**
- **보드레이트 115200 설정**

---

### 2️⃣ **IMU 및 엔코더 데이터 수신 (`readSpeed` 메서드)**
```python
def readSpeed(self):
    buf_header = bytearray(1)
    buf = bytearray(27)
    self.ser.readinto(buf_header)
    if buf_header[0] == 0xf5:
        self.ser.readinto(buf_header)
        if buf_header[0] == 0xf5:
            self.ser.readinto(buf)
            self.ax = struct.unpack('>h', buf[0:2])[0]/1000
            self.ay = struct.unpack('>h', buf[2:4])[0]/1000
            self.az = struct.unpack('>h', buf[4:6])[0]/1000
```
- **2바이트 헤더 확인 (`0xF5 0xF5`)**
- **IMU 데이터 (가속도, 자이로, 자기장) 파싱**
- **엔코더 데이터 파싱**

---

### 3️⃣ **모터 제어 (`move_run_mode` 메서드)**
```python
def move_run_mode(self ,run_mode, speed):
    buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, run_mode, speed])
    self.ser.write(buf)
```
- **로봇을 전진/후진/좌회전/우회전/정지 시킴**
- **패킷을 생성하여 Arduino로 전송**

사용 예시:
```python
bot = Jdamr200("/dev/ttyACM0")
bot.move_run_mode(bot.GO_FORWARD, 100)
```

---

### 4️⃣ **Thread를 사용한 데이터 수신 (`receive_thread` 메서드)**
```python
def receive_thread(self):
    rx_task = threading.Thread(target=self.receive_data)
    rx_task.setDaemon(True)
    rx_task.start()
```
- **데이터 수신을 위한 별도 스레드 실행**
- **모터 제어와 동시에 센서 데이터를 지속적으로 수신 가능**

---

### 5️⃣ **Odometry 업데이트 (`update_odometry` 메서드)**
```python
def update_odometry(self):
    delta_left = self.lf_encoder - self.previous_left_encoder
    delta_right = self.rf_encoder - self.previous_right_encoder
    left_distance = (2 * math.pi * self.wheel_radius) * (delta_left / self.encoder_resolution)
    right_distance = (2 * math.pi * self.wheel_radius) * (delta_right / self.encoder_resolution)
    delta_distance = (left_distance + right_distance) / 2.0
    delta_theta = (right_distance - left_distance) / self.wheel_base
    self.x += delta_distance * math.cos(self.theta)
    self.y += delta_distance * math.sin(self.theta)
    self.theta += delta_theta
```
- **엔코더 값을 바탕으로 Odometry 업데이트**
- **위치 (`x`, `y`), 회전각 (`theta`) 계산**

---

## 🏁 실행 예제
JDAMR200 Python 라이브러리를 사용하여 로봇을 제어하는 예제입니다.

```python
from jdamr200_lib import Jdamr200
bot = Jdamr200("/dev/ttyACM0")
bot.receive_thread()  # 센서 데이터 수신 스레드 실행

time.sleep(2)

bot.move_run_mode(bot.GO_FORWARD, 100)  # 전진
bot.move_run_mode(bot.GO_BACKWARD, 100) # 후진
bot.move_run_mode(bot.STOP, 0)          # 정지
```

---

## 🎯 결론
이 Python 라이브러리는 **JDAMR200 로봇을 제어**하는 핵심적인 기능을 제공합니다.
- **시리얼 통신을 통한 모터 제어**
- **IMU 및 엔코더 데이터 수신**
- **ROS2를 위한 Odometry 업데이트**
- **Thread를 활용한 병렬 처리 지원**

자세한 내용은 [GitHub 저장소](https://github.com/JD-edu/jdamr200)를 참고하세요! 🚀

