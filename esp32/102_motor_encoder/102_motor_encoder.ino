#include "TB6612Motor.h"
#include <cmath> 
#include <Arduino.h> // ESP32 함수를 위해 명시적으로 포함

// --- --- --- (엔코더 부분) --- --- ---

// A 휠 엔코더 
const uint16_t AENCA = 35;         // Encoder A input A_C2(B) - 방향 판단용
const uint16_t AENCB = 34;         // Encoder A input A_C1(A) - 인터럽트 핀

// B 휠 엔코더 
const uint16_t BENCB = 16;         // Encoder B input B_C2(B) - 인터럽트 핀
const uint16_t BENCA = 27;         // Encoder B input B_C1(A) - 방향 판단용

volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count  = 0;

// 속도 계산 주기 (엔코더 출력 주기만 사용)
int interval = 100; 

// 중단 함수 (엔코더 펄스 처리 - B 모터)
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){ // BENCA 핀 상태로 방향 판단
    B_wheel_pulse_count++;
  }
  else{
    B_wheel_pulse_count--;
  }
}

// 중단 함수 (엔코더 펄스 처리 - A 모터)
void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){ // AENCA 핀 상태로 방향 판단
    A_wheel_pulse_count++;
  }
  else{
    A_wheel_pulse_count--;
  }
}

// --- --- --- 电机控制部分 (모터 제어 부분) --- --- ---

// A 모터제어 핀 정의
const uint16_t PWMA = 25; 
const uint16_t AIN2 = 17; 
const uint16_t AIN1 = 21; 
const int channel_A = 5;

// B 모터제어 핀 정의
const uint16_t BIN1 = 22; 
const uint16_t BIN2 = 23; 
const uint16_t PWMB = 26; 
const int channel_B = 6;

// 모터 구동 속도 (PWM)
const float MOTOR_SPEED_PWM = 127.0; // 8비트 해상도 (0-255)의 중간 속도

// PWM 설정 (클래스 생성자에 전달할 값)
const int freq = 100000;
const uint16_t ANALOG_WRITE_BITS = 8;

// 모터 클래스 인스턴스 생성
TB6612Motor motorA(PWMA, AIN1, AIN2, channel_A, freq, ANALOG_WRITE_BITS);
TB6612Motor motorB(PWMB, BIN1, BIN2, channel_B, freq, ANALOG_WRITE_BITS);


void setup(){
  // --- 엔코더 핀 설정 ---
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);
  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
  
  // 엔코더 인터럽트 설정: BENCB (B_C2)와 AENCB (A_C1) 핀의 RISING 엣지에 연결
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  // 시리얼 통신 초기화
  Serial.begin(115200);
  while(!Serial){}

  // --- 모터 클래스 초기화 ---
  motorA.begin();
  motorB.begin();
  
  // 모터 구동 시작 (중간 속도 127)
  motorA.control(MOTOR_SPEED_PWM);
  motorB.control(MOTOR_SPEED_PWM);
}


void loop(){
  // 엔코더 펄스 수만 시리얼 출력
  // (PID, 속도 계산, PID 변수, 불필요한 전역 변수 모두 제거됨)
  
  Serial.print("Count B: "); Serial.print(B_wheel_pulse_count);
  Serial.print(" | Count A: "); Serial.println(A_wheel_pulse_count);

  // 출력 주기를 유지 (원본 코드의 interval 100ms 사용)
  delay(interval);
}