// Operate the STS3215 servo in velocity mode
//
// This code makes the motor spin continuously at a given velocity

#include "STSServoDriver.h"

STSServoDriver servos;

// ID of the servo currently being tested.
byte SERVO_ID = 3;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);

  // Since the serial port is taken by the servo, we can't easily send debug messages, so
  // we use the on-board led instead.
  // Try to connect with the servos, using pin 2 as direction pin and the default (only) serial
  // interface of an Arduino Uno.
  if (!servos.init(2, &Serial1))
  {
    // Failed to get a ping reply, turn on the led.
    digitalWrite(13, HIGH);
  }
  // Set the servo to velocity mode.
  servos.setMode(SERVO_ID, STSMode::VELOCITY);

  // Spin clockwise at 500steps/s, i.e one turn every 8 seconds
  servos.setTargetVelocity(SERVO_ID, 500);
  delay(4000);
  // Spin counter-clockwise at 2048steps/s, i.e one turn every 2 seconds
  servos.setTargetVelocity(SERVO_ID, -2048);
  delay(2000);//servos.setTargetVelocity(SERVO_ID, 0);
  servos.setTargetVelocity(SERVO_ID, 0);
  delay(2000);
  servos.setTorque(3, true); // ID가 1인 서보의 토크를 끕니다.
}

void loop()
{

  float current = servos.getCurrentCurrent(SERVO_ID);
  Serial.print("Current reading (A): ");
  Serial.println(current, 4);
  delay(100);
}
