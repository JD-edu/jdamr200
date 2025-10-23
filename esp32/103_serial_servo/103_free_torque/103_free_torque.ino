// Feetech 서보를 이용한 임시 원점 설정 및 제어 예제

#include "STSServoDriver.h"

// STSServoDriver 클래스의 전역 인스턴스
STSServoDriver servos;

// 제어할 서보의 ID
byte SERVO_ID = 3;

// 1도에 해당하는 서보 위치값 (4096단계 / 360도)
const float STEPS_PER_DEGREE = 4096.0 / 360.0;

// 임시 원점 위치를 저장할 변수
int temporaryOriginPosition = 0;

void setup() {
  // 시리얼 통신 초기화 (PC와 통신)
  Serial.begin(115200); 

  // 온보드 LED 핀 설정 (ESP32의 경우 GPIO 13)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);

  // STSServoDriver 초기화
  // dirPin: 2, serialPort: &Serial2 (ESP32용), baudRate: 1000000 (기본값)
  if (!servos.init(2, &Serial1)) {
    Serial.println("Servo initialization failed. Check connections and power.");
    digitalWrite(13, HIGH); // 실패 시 LED 켜기
    while(true); 
  } else {
    Serial.println("Servo initialization successful.");
  }

  // 서보를 위치 제어 모드(POSITION mode)로 설정
  servos.setMode(SERVO_ID, STSMode::POSITION);

  Serial.println("\n-----------------------------------------------------");
  Serial.println("Feetech Servo Control with Temporary Origin");
  Serial.println("Commands:");
  Serial.println("  'a' : Turn torque OFF (Allow manual rotation)");
  Serial.println("  'b' : Set current position as temporary origin and turn torque ON");
  Serial.println("  'c' : Move to 0 degrees from temporary origin");
  Serial.println("  'd' : Move to 90 degrees from temporary origin");
  Serial.println("  'e' : Move to 180 degrees from temporary origin");
  Serial.println("-----------------------------------------------------");
  
  // 시작 시 토크를 켜서 서보를 제어 가능 상태로 만듭니다.
  servos.setTorque(SERVO_ID, true);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    int targetPosition = 0;
    
    switch (command) {
      case 'a':
        Serial.println("Command 'a': Turning torque OFF. You can now manually rotate the servo.");
        servos.setTorque(SERVO_ID, false);
        break;
      
      case 'b':
        temporaryOriginPosition = servos.getCurrentPosition(SERVO_ID);
        servos.setTorque(SERVO_ID, true);
        Serial.print("Command 'b': Torque ON. Temporary origin set to absolute position: ");
        Serial.println(temporaryOriginPosition);
        break;
        
      case 'c':
        targetPosition = temporaryOriginPosition;
        Serial.print("Command 'c': Moving to 0 degrees. Target absolute position: ");
        Serial.print(" ");
        Serial.print(temporaryOriginPosition);
        Serial.print(" ");
        Serial.println(targetPosition);
        servos.setTargetPosition(SERVO_ID, targetPosition);
        break;
        
      case 'd':
        targetPosition = temporaryOriginPosition + (90 * STEPS_PER_DEGREE);
        Serial.print("Command 'd': Moving to 90 degrees. Target absolute position: ");
        Serial.print(" ");
        Serial.print(temporaryOriginPosition);
        Serial.print(" ");
        servos.setTargetPosition(SERVO_ID, targetPosition);
        break;
        
      case 'e':
        targetPosition = temporaryOriginPosition + (180 * STEPS_PER_DEGREE);
        Serial.print("Command 'e': Moving to 180 degrees. Target absolute position: ");
        Serial.print(" ");
        Serial.print(temporaryOriginPosition);
        Serial.print(" ");
        servos.setTargetPosition(SERVO_ID, targetPosition);
        break;
        
      default:
        Serial.println("Unknown command. Please enter a, b, c, d, or e.");
        break;
    }
  }
}