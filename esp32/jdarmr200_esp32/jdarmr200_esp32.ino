/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/

#include "TB6612Motor.h"
#include <Adafruit_SSD1306.h>
// install following library
// https://github.com/adafruit/Adafruit_SSD1306 
#include <INA219_WE.h>
// install following library
// https://github.com/wollewald/INA219_WE 

// A路电机 (Motor A) 핀 정의
const uint16_t PWMA = 25; 
const uint16_t AIN2 = 17; 
const uint16_t AIN1 = 21; 
const int CHANNEL_A = 5;
int speed_a = 0;

// B路电机 (Motor B) 핀 정의
const uint16_t PWMB = 26; 
const uint16_t BIN1 = 22; 
const uint16_t BIN2 = 23; 
const int CHANNEL_B = 6;
int speed_b = 0;

#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

#define S_SCL   33
#define S_SDA   32

// 모터 A 인스턴스 생성
TB6612Motor motorA(PWMA, AIN1, AIN2, CHANNEL_A);
// 모터 B 인스턴스 생성
TB6612Motor motorB(PWMB, BIN1, BIN2, CHANNEL_B);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {
  // 각 모터 초기화
  motorA.begin();
  motorB.begin();
  Wire.begin(S_SDA,S_SCL);
  Serial.begin(115200);
  InitScreen();
  InitINA219();
  Serial.println("jdAMR200 start...");
}


void loop() {
  Screenupdate();
  InaDataUpdate();
  allDataUpdate();
  delay(100);

  motorA.control(speed_a);
  motorB.control(speed_b);
  speed_a += 1;
  speed_b += 1;
  if(speed_a > 255){
    speed_a = 50;
    speed_b = 50;
  }

}

void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void Screenupdate(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(F("speed A:"));
  display.setCursor(80, 0);
  display.print(speed_a);
  display.setCursor(0,10);
  display.print(F("speed B:"));
  display.setCursor(80, 10);
  display.print(speed_b);
  display.setCursor(0,20);
  display.print(F("Battery:"));
  display.setCursor(80, 20);
  display.print(loadVoltage_V);
  display.display();
}

void InitINA219(){
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(INA219_BIT_MODE_9);
  ina219.setPGain(INA219_PG_320);
  ina219.setBusRange(INA219_BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}

void allDataUpdate(){
    Serial.print("battery:");
    Serial.println(loadVoltage_V);
    Serial.print("current_mA:");
    Serial.println(current_mA);
}

