#include "TB6612Motor.h"
#include <cmath> // round 함수를 위해 필요

// 생성자
TB6612Motor::TB6612Motor(uint16_t pwmPin, uint16_t in1Pin, uint16_t in2Pin,
                         int channel, int freq, uint8_t bits)
    : _PWMPin(pwmPin), _IN1Pin(in1Pin), _IN2Pin(in2Pin),
      _channel(channel), _freq(freq), _ANALOG_WRITE_BITS(bits)
{
    // MAX_PWM 및 MIN_PWM 값 계산
    _MAX_PWM = static_cast<uint16_t>(pow(2, _ANALOG_WRITE_BITS) - 1);
    // MIN_PWM: MAX_PWM/5
    _MIN_PWM = _MAX_PWM / 5;
}

// 초기화
void TB6612Motor::begin() {
    // 제어 핀 모드 설정
    pinMode(_IN1Pin, OUTPUT);
    pinMode(_IN2Pin, OUTPUT);
    pinMode(_PWMPin, OUTPUT);

    // PWM 설정 및 핀 연결 (ESP32 LEDC)
    ledcSetup(_channel, _freq, _ANALOG_WRITE_BITS);
    ledcAttachPin(_PWMPin, _channel);

    // 초기 상태: 모터 정지
    digitalWrite(_IN1Pin, LOW);
    digitalWrite(_IN2Pin, LOW);
    ledcWrite(_channel, 0);
}

// 모터 제어
void TB6612Motor::control(float pwmInput) {
    // 부동 소수점 PWM 값을 정수로 반올림
    int pwmInt = round(pwmInput);

    if (pwmInt == 0) {
        // 정지
        stop();
        return;
    }

    if (pwmInt > 0) {
        // 정방향 (AIN1/BIN1 = LOW, AIN2/BIN2 = HIGH)
        digitalWrite(_IN1Pin, LOW);
        digitalWrite(_IN2Pin, HIGH);
        // MIN_PWM ~ MAX_PWM 범위로 제한
        ledcWrite(_channel, constrain(pwmInt, _MIN_PWM, _MAX_PWM));
    } else {
        // 역방향 (AIN1/BIN1 = HIGH, AIN2/BIN2 = LOW)
        digitalWrite(_IN1Pin, HIGH);
        digitalWrite(_IN2Pin, LOW);
        // 절대값을 취하고 MIN_PWM ~ MAX_PWM 범위로 제한
        // constrain()은 -MAX_PWM ~ -MIN_PWM으로 제한 후 음수 부호를 제거
        ledcWrite(_channel, -constrain(pwmInt, -_MAX_PWM, -_MIN_PWM));
    }
}

// 모터 정지
void TB6612Motor::stop() {
    digitalWrite(_IN1Pin, LOW);
    digitalWrite(_IN2Pin, LOW);
    ledcWrite(_channel, 0); // PWM 0
}