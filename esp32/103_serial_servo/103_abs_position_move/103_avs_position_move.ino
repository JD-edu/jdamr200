// Operate the STS3215 servo in position mode to rotate 360 degrees.

#include "STSServoDriver.h"

// STSServoDriver 클래스의 전역 인스턴스
STSServoDriver servos;

// 제어할 서보의 ID. 이 예시에서는 3번 ID를 사용합니다.
byte SERVO_ID = 3;

// 10도에 해당하는 서보 위치값 (4096 / 360 * 10 = 약 114)
const int POSITION_PER_10_DEGREES = 114;

void setup() {
  // 시리얼 통신 초기화 (디버깅 메시지 출력용)
  Serial.begin(115200); 

  // 온보드 LED 핀 설정 (ESP32의 경우 일반적으로 GPIO 13)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // STSServoDriver 초기화
  if (!servos.init(2, &Serial2)) {
    Serial.println("Servo initialization failed. Check connections and power.");
    digitalWrite(13, HIGH); // 실패 시 LED 켜기
    while(true); 
  } else {
    Serial.println("Servo initialization successful.");
  }

  // 서보의 토크를 활성화합니다.
  servos.setTorque(SERVO_ID, true);
  
  // 서보를 위치 제어 모드(POSITION mode)로 설정합니다.
  servos.setMode(SERVO_ID, STSMode::POSITION);

  Serial.println("Servo is now in position mode.");
}

void loop() {
  Serial.println("Starting 360-degree rotation (10 degrees at a time)...");

  // 0도에서 350도까지 10도씩 이동
  for (int i = 0; i < 36; i++) {
    // 10도에 해당하는 위치 값을 곱하여 목표 위치를 계산
    int targetPosition = i * POSITION_PER_10_DEGREES;
    
    // 서보를 목표 위치로 이동시킵니다.
    servos.setTargetPosition(SERVO_ID, targetPosition);
    Serial.print("Moving to position: ");
    Serial.println(targetPosition);

    // 서보가 이동할 시간을 줍니다. (필요에 따라 조절)
    delay(200); 
  }

  // 마지막으로 원점(0)으로 돌아옵니다.
  servos.setTargetPosition(SERVO_ID, 0);
  Serial.println("Returning to origin (0)...");
  delay(2000); // 원점으로 돌아오는 데 더 긴 시간을 줍니다.

  Serial.println("Rotation cycle complete. Restarting in 5 seconds.");
  delay(5000); // 다음 사이클 시작 전 5초 대기
}