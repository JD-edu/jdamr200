#include "TB6612Motor.h"

// A路电机 (Motor A) 핀 정의
const uint16_t PWMA = 25; 
const uint16_t AIN2 = 17; 
const uint16_t AIN1 = 21; 
const int CHANNEL_A = 5;

// B路电机 (Motor B) 핀 정의
const uint16_t PWMB = 26; 
const uint16_t BIN1 = 22; 
const uint16_t BIN2 = 23; 
const int CHANNEL_B = 6;


// 모터 A 인스턴스 생성
TB6612Motor motorA(PWMA, AIN1, AIN2, CHANNEL_A);

// 모터 B 인스턴스 생성
TB6612Motor motorB(PWMB, BIN1, BIN2, CHANNEL_B);


void setup() {
  // 각 모터 초기화
  motorA.begin();
  motorB.begin();
}


void loop() {
  // 1. 두 모터 모두 정지 3초
  motorA.stop();
  motorB.stop();
  delay(3000);

  // 2. 모터 역방향, 최고 속도 (-255) 3초
  motorA.control(-255);
  motorB.control(-255);
  delay(3000);

  // 3. 모터 정방향, 최고 속도 (255) 3초
  motorA.control(255);
  motorB.control(255);
  delay(3000);
}