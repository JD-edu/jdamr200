// Feetech 서보를 위한 외부 PID 위치 제어 예제

#include "STSServoDriver.h"

// STSServoDriver 클래스의 전역 인스턴스
STSServoDriver servos;

// 제어할 서보의 ID
byte SERVO_ID = 3;

// 1도에 해당하는 서보 위치값 (4096단계 / 360도)
const float STEPS_PER_DEGREE = 4096.0 / 360.0;

// PID 제어 상수
const float Kp = 0.5;
const float Ki = 0.01;
const float Kd = 0.1;

// PID 변수
int targetPosition = 1024;
int lastError = 0;
float integral = 0;

unsigned long previousMillis = 0;
const long interval = 5000; // 5초마다 목표 위치 변경

void setup() {
  // 시리얼 통신 초기화 (PC와 통신)
  Serial.begin(115200); 

  // 온보드 LED 핀 설정 (ESP32의 경우 GPIO 13)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);

  // STSServoDriver 초기화
  if (!servos.init(2, &Serial1)) {
    Serial.println("Servo initialization failed. Check connections and power.");
    digitalWrite(13, HIGH);
    while(true); 
  } else {
    Serial.println("Servo initialization successful.");
  }

  // 서보의 토크를 활성화하고, 속도 모드로 설정
  servos.setTorque(SERVO_ID, true);
  servos.setMode(SERVO_ID, STSMode::VELOCITY);

  Serial.println("External PID control loop for position. Starting...");
}

void loop() {
  unsigned long currentMillis = millis();

  // 5초마다 목표 위치 변경
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (targetPosition == 1024) {
      targetPosition = 3072;
    } else {
      targetPosition = 1024;
    }
    Serial.print("Target position updated to: ");
    Serial.println(targetPosition);
  }

  // 현재 위치 읽기
  int currentPosition = servos.getCurrentPosition(SERVO_ID);
  
  // 오차 계산
  int error = targetPosition - currentPosition;
  
  // PID 계산
  integral += error;
  int derivative = error - lastError;
  int output = Kp * error + Ki * integral + Kd * derivative;
  
  // 서보의 속도 제어
  servos.setTargetVelocity(SERVO_ID, output);
  
  // 다음 루프를 위해 현재 오차를 저장
  lastError = error;

  // 디버깅 정보 출력
  Serial.print("Target: ");
  Serial.print(targetPosition);
  Serial.print(", Current: ");
  Serial.print(currentPosition);
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", Output Velocity: ");
  Serial.println(output);

  delay(10); // PID 제어 루프 주기
}