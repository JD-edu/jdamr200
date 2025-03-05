# JDAMR200 Arduino 펌웨어 문서

## 개요
이 펌웨어는 **Arduino Mega2560**과 **TB6612FNG 모터 드라이버**, **MPU9250 IMU 센서**를 사용하여 **자율 주행 로봇(AMR)**을 제어하도록 설계되었습니다. 
이 펌웨어는 **4개의 바퀴를 제어**하고, **IMU 및 엔코더 데이터를 수집**하며, 이 데이터를 **Raspberry Pi로 전송**합니다.

---

## 📂 파일 구조
```
arduino/
├── jdamr200_control.ino   # 메인 Arduino 펌웨어
├── tb6612_motor.cpp       # TB6612FNG 모터 드라이버 구현 파일
├── tb6612_motor.h         # TB6612FNG 모터 드라이버 헤더 파일
├── Encoder/               # 외부 엔코더 라이브러리
└── MPU9250/               # 외부 MPU9250 라이브러리
```

---

## 🔧 모터 드라이버 (TB6612FNG)
**TB6612FNG 모터 드라이버**를 사용하여 4개의 **DC 모터**를 제어합니다. PWM 신호와 방향 핀을 설정하여 동작합니다.

모터 객체는 다음과 같이 생성됩니다:
```cpp
SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 35, 34);
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 37, 36);
SPDMotor *motorLR = new SPDMotor(3, 49, true, 6, 42, 43);
SPDMotor *motorRR = new SPDMotor(2, A1, false, 5, A4, A5);
```
각 모터는 다음과 같은 핀을 할당받습니다:
- **PWM 핀**: 속도 제어
- **방향 제어 핀**
- **엔코더 핀**: 위치 피드백

---

## 🚦 LED 상태 표시
디버깅을 위해 **RGB LED**를 사용하여 시스템 상태를 표시합니다:
- **`imu_error()`**: MPU9250 초기화 실패 시 빨간 LED 점멸
- **`reboot_alarm()`**: 부팅 시 녹색 LED 점멸
- **`arduino_working()`**: 정상 동작 시 파란 LED 토글

```cpp
void imu_error(){
  analogWrite(A6, 255);
  delay(100);
  analogWrite(A6, 0);
  delay(100);
}
```

---

## 🚗 모터 제어 함수
기본적인 모터 제어 함수입니다:
```cpp
void go_forward(int speed){
    motorLF->speed(speed); 
    motorLR->speed(speed); 
    motorRF->speed(speed);
    motorRR->speed(speed);
}
void turn_left(int speed){
    motorLF->speed(-speed); 
    motorLR->speed(-speed); 
    motorRF->speed(speed);
    motorRR->speed(speed);
}
```
- **`go_forward(speed)`**: 로봇을 전진시킵니다.
- **`turn_left(speed)`**: 로봇을 왼쪽으로 회전시킵니다.
- **`stop()`**: 모든 모터를 정지합니다.

---

## 🔄 Arduino `setup()` 함수
```cpp
void setup(){
    Serial.begin(115200);
    Serial2.begin(115200);  // 디버깅 용도
    Wire.begin();
    delay(2000);
    if (!mpu.setup(0x68)) {
        while (1) { imu_error(); }
    }
    reboot_alarm();
}
```
- **시리얼 포트**:
  - `Serial`: Raspberry Pi와 통신용
  - `Serial2`: 디버깅 용도
- **MPU9250 초기화**: I2C 통신을 통해 설정
- **부팅 LED 알림**: 부팅 완료 표시

---

## 📡 Raspberry Pi → Arduino 패킷 프로토콜
Arduino는 시리얼 통신을 통해 Raspberry Pi로부터 모터 제어 패킷을 수신합니다.
### **패킷 형식**
```
[0xF5] [0xF5] [Command] [Direction] [Speed]
```
### **패킷 파싱 방법**
```cpp
if(Serial.available() > 0){
    Serial.readBytes(cmd_buf, 5);
    if(cmd_buf[0] == 0xf5 && cmd_buf[1] == 0xf5){
        int speed = (int)cmd_buf[4];
        if(cmd_buf[3] == GO_FORWARD){
            go_forward(speed);
        }else if(cmd_buf[3] == STOP){
            stop();
        }
    }
}
```

---

## 📡 MPU9250 & 엔코더 데이터 전송 (Arduino → Raspberry Pi)
센서 데이터는 **바이너리 형식**으로 전송됩니다.

### **패킷 구조 (29 바이트)**
```
[0xF5] [0xF5] [IMU 데이터 (6바이트)] [자이로 데이터 (6바이트)] [자력계 데이터 (6바이트)] [엔코더 데이터 (8바이트)] [체크섬]
```

### **센서 데이터 전송 함수**
```cpp
void send_sensors(){
    byte data[29] = {0};
    data[0] = 0xf5;
    data[1] = 0xf5;
    data[2] = (ax_l >> 8) & 0xff;
    data[3] = ax_l & 0xff;
    data[20] = (lf_encoder >> 8) & 0xff;
    data[21] = lf_encoder & 0xff;
    Serial.write(data, 29);
}
```

---

## 📥 필요한 라이브러리
다음 라이브러리를 설치해야 합니다:
1. **Encoder 라이브러리** ([다운로드](https://github.com/PaulStoffregen/Encoder))
2. **MPU9250 라이브러리** ([다운로드](https://github.com/hideakitai/MPU9250))

Arduino Library Manager를 통해 설치하거나 `libraries/` 폴더에 수동으로 추가하세요.

---

## 🎯 결론
이 Arduino 펌웨어는 다음 기능을 제공합니다:
- **TB6612FNG 모터 드라이버를 이용한 모터 제어**
- **MPU9250 센서 데이터를 Raspberry Pi로 전송**
- **시리얼 통신을 통한 원격 제어**

자세한 내용은 [GitHub 저장소](https://github.com/JD-edu/jdamr200)를 참고하세요.

