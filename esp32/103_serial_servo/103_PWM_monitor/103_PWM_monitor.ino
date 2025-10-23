#include "STSServoDriver.h"

// STSServoDriver 클래스의 전역 인스턴스
STSServoDriver servos;

// 제어할 서보의 ID. 이 예시에서는 3번 ID를 사용합니다.
byte SERVO_ID = 3;

void setup() {
  // 시리얼 통신 초기화 (디버깅 메시지 출력용)
  // ESP32의 USB-UART 통신을 사용합니다.
  Serial.begin(115200); 

  // 온보드 LED 핀 설정 (ESP32의 경우 일반적으로 GPIO 2 또는 13)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);

  // STSServoDriver 초기화
  // dirPin: 2, serialPort: &Serial2 (ESP32용), baudRate: 1000000 (기본값)
  if (!servos.init(2, &Serial1)) {
    Serial.println("Servo initialization failed. Check connections and power.");
    digitalWrite(13, HIGH); // 실패 시 LED 켜기
    // 루프에 진입하지 않도록 여기서 대기
    while(true); 
  } else {
    Serial.println("Servo initialization successful.");
  }

  // 서보의 토크를 활성화합니다.
  servos.setTorque(SERVO_ID, true);
  
  // 서보를 위치 제어 모드(POSITION mode)로 설정합니다.
  servos.setMode(SERVO_ID, STSMode::POSITION);

  Serial.println("Servo is now in position mode.");
  // 목표 위치 0으로 이동 (예시: 최소 각도)
  servos.setTargetPosition(SERVO_ID, 0);
  Serial.println("Moving to position 0...");
  delay(2000); // 서보가 움직일 시간을 줍니다.
   // 목표 위치 1023으로 이동 (예시: 최대 각도)
  servos.setTargetPosition(SERVO_ID, 1023);
  Serial.println("Moving to position 1023...");
  delay(2000); // 서보가 움직일 시간을 줍니다.
}

void loop() {
  
 
  signed char pwmValue = servos.getCurrentPWM(SERVO_ID);
    Serial.print("Raw PWM Value: ");
    Serial.println(pwmValue); // Print the raw byte value in decimal format 
    delay(100);
}