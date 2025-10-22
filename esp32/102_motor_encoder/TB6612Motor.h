#ifndef TB6612MOTOR_H
#define TB6612MOTOR_H

#include <Arduino.h>

/**
 * @brief TB6612FNG 듀얼 DC 모터 드라이버의 단일 모터 채널 제어 클래스.
 *
 * 이 클래스는 TB6612FNG 칩에 연결된 하나의 DC 모터를
 * ESP32의 PWM 기능을 이용해 제어합니다.
 */
class TB6612Motor {
public:
    /**
     * @brief TB6612Motor 클래스의 생성자.
     * @param pwmPin 모터의 PWM 제어 핀 (예: 25 또는 26)
     * @param in1Pin 모터의 입력 1 핀 (예: 21 또는 22)
     * @param in2Pin 모터의 입력 2 핀 (예: 17 또는 23)
     * @param channel ESP32 LEDC 채널 번호 (예: 5 또는 6)
     * @param freq PWM 주파수 (Hz) (기본값: 100000)
     * @param bits PWM 해상도 비트 수 (기본값: 8, 0-255)
     */
    TB6612Motor(uint16_t pwmPin, uint16_t in1Pin, uint16_t in2Pin,
                int channel, int freq = 100000, uint8_t bits = 8);

    /**
     * @brief 모터 초기 설정 (setup).
     * 핀 모드 설정 및 PWM 채널 설정을 수행합니다.
     */
    void begin();

    /**
     * @brief 모터 제어.
     * @param pwmInput 부동 소수점 PWM 값 (-MAX_PWM ~ +MAX_PWM).
     * 양수: 정방향, 음수: 역방향, 0: 정지.
     */
    void control(float pwmInput);

    /**
     * @brief 모터 정지.
     */
    void stop();

private:
    // 핀 및 채널 설정
    const uint16_t _PWMPin;
    const uint16_t _IN1Pin;
    const uint16_t _IN2Pin;
    const int _channel;

    // PWM 설정
    const int _freq;
    const uint8_t _ANALOG_WRITE_BITS;

    // PWM 값
    uint16_t _MAX_PWM;
    uint16_t _MIN_PWM;
};

#endif // TB6612MOTOR_H