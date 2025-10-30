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

// ---------------------- Header file ----------------------- //
#include "TB6612Motor.h"
#include <Adafruit_SSD1306.h>
// install following library
// https://github.com/adafruit/Adafruit_SSD1306 
#include <INA219_WE.h>
// install following library
// https://github.com/wollewald/INA219_WE 
#include "IMU.h"

// ---------------------------------------------------------- //
// Motor Setup 
// ---------------------------------------------------------- //
// Motor A 핀 정의
const uint16_t PWMA = 25; 
const uint16_t AIN2 = 17; 
const uint16_t AIN1 = 21; 
const int CHANNEL_A = 5;
int speed_a = 0;
// Motor B 핀 정의
const uint16_t PWMB = 26; 
const uint16_t BIN1 = 22; 
const uint16_t BIN2 = 23; 
const int CHANNEL_B = 6;
int speed_b = 0;
// 모터 A 인스턴스 생성
TB6612Motor motorA(PWMA, AIN1, AIN2, CHANNEL_A);
// 모터 B 인스턴스 생성
TB6612Motor motorB(PWMB, BIN1, BIN2, CHANNEL_B);
// motor command packet and length 
const byte COMMAND_PACKET_LENGTH = 5;
byte cmd_buf[COMMAND_PACKET_LENGTH];
// Run Mode 명령어 정의
#define GO_FORWARD    1
#define GO_BACKWARD   2 
#define TURN_LEFT     3
#define TURN_RIGHT    4
#define STOP          5
#define CMD_RESET     6
// motor direction 
int direction = 0; // Direction (0x01 ~ 0x05)
// motor speed 
int speed = 0;     // Speed (0x00 ~ 0xFF)

// ---------------------------------------------------------- //
// Encoder Setup 
// ---------------------------------------------------------- //
// A 휠 엔코더 
const uint16_t AENCA = 35;         // Encoder A input A_C2(B) - 방향 판단용
const uint16_t AENCB = 34;         // Encoder A input A_C1(A) - 인터럽트 핀
// B 휠 엔코더 
const uint16_t BENCB = 16;         // Encoder B input B_C2(B) - 인터럽트 핀
const uint16_t BENCA = 27;         // Encoder B input B_C1(A) - 방향 판단용
// 엔코더 밸류 (2바이트 정수)
int lf_encoder = 0;
int rf_encoder = 0;
int lr_encoder = 0;
int rr_encoder = 0;

// ---------------------------------------------------------- //
// Voltage and Current Meter 
// ---------------------------------------------------------- //
// INA219 I2C address
#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

// ---------------------------------------------------------- //
// I2C bus pin assign
// ---------------------------------------------------------- //
#define S_SCL   33
#define S_SDA   32

// ---------------------------------------------------------- //
// IMU Setup
// ---------------------------------------------------------- //
// Eulaer angle data struct 
EulerAngles stAngles;
// Gyro sensor data struct
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
// Accel sensor data struct
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
// Magnet sensor data struct 
IMU_ST_SENSOR_DATA stMagnRawData;
// Temperature 
float temp;
// 센서 데이터 송신 패킷 정의 (29 Bytes)
// 헤더(2) + 가속도(3x2) + 자이로(3x2) + 마그네틱(3x2) + 엔코더(4x2) + 체크섬(1) = 29 bytes
const byte SENSOR_PACKET_LENGTH = 29;
byte sensor_data_packet[SENSOR_PACKET_LENGTH] = {0,};
// 센서 데이터 변수 
int ax_l = 0;   // Accel X (가상값)
int ay_l = 0;   // Accel Y
int az_l = 0;   // Accel Z (대략 1G * 1000)
int gx_l = 0;   // Gyro X
int gy_l = 0;   // Gyro Y
int gz_l = 0;   // Gyro Z
int mx_l = 0;   // Magnetic X
int my_l = 0;   // Magnetic Y
int mz_l = 0;   // Magnetic Z

// ---------------------------------------------------------- //
// OLED Setup 
// ---------------------------------------------------------- //
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET     -1   // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// OLED 출력 버퍼 (디버그 메시지를 저장하고 화면에 표시하기 위함)
String debug_line_1 = "        ";
String debug_line_2 = "        ";
String debug_line_3 = "        ";

// ----------------- Encoder interrupt  --------------------- //
// interrupt handler (엔코더 펄스 처리 - B 모터)
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){ // BENCA 핀 상태로 방향 판단
    lf_encoder++;
  }
  else{
    lf_encoder--;
  }
}
// interrupt handler (엔코더 펄스 처리 - A 모터)
void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){ // AENCA 핀 상태로 방향 판단
    rf_encoder++;
  }
  else{
    rf_encoder--;
  }
}

// ---------------------------------------------------------- //
// setup function 
// ---------------------------------------------------------- //
void setup() {
  // 각 모터 초기화
  motorA.begin();
  motorB.begin();
  // --- 엔코더 핀 설정 ---
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);
  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
  // 엔코더 인터럽트 설정: BENCB (B_C2)와 AENCB (A_C1) 핀의 RISING 엣지에 연결
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);
  // I2C start 
  Wire.begin(S_SDA,S_SCL);
  Serial.begin(115200);
  // Volt current meter start  
  InitINA219();
  // IMU sensor start 
  imuInit();
  // OLED start 
  InitScreen();
}

// ---------------------------------------------------------- //
// loop function 
// ---------------------------------------------------------- //
void loop() {
  update_oled_display();
  InaDataUpdate();
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  // 1. 시리얼 포트(Serial/ROS)에 5바이트 이상의 데이터가 있는지 확인
  if(Serial.available() >= 5) { 
    // 2. 명령 패킷 5바이트를 수신 버퍼로 읽어 들입니다.
    // Serial.readBytes는 수신 가능한 바이트가 부족하면 타임아웃까지 대기합니다.
    size_t bytesRead = Serial.readBytes(cmd_buf, 5); 
    // 3. 읽은 바이트 수가 5인지 확인하고 패킷을 해석합니다.
    if (bytesRead == 5) {   
      // 4. 헤더 체크: 0xF5, 0xF5
      if(cmd_buf[0] == 0xf5 && cmd_buf[1] == 0xf5) {
        // 5. 모드 체크: Run Mode (0x51)?
        if(cmd_buf[2] == 0x51) {
          int direction = (int)cmd_buf[3]; // Direction (0x01 ~ 0x05)
          int speed = (int)cmd_buf[4];     // Speed (0x00 ~ 0xFF)
          // 6. 방향에 따른 모터 제어 함수 호출
          if(direction == GO_FORWARD){
              debug_line_2 = "motor: FOWARD ";   
              go_forward(speed);     
          } else if(direction == GO_BACKWARD){
              debug_line_2 = "motor: BACK   ";
              go_backward(speed);
          } else if(direction == TURN_LEFT){
              debug_line_2 = "motor: LEFT   ";
              turn_left(speed);
          } else if(direction == TURN_RIGHT){
              debug_line_2 = "motor: RIGHT  ";
              turn_right(speed);
          } else if(direction == STOP){
              debug_line_2 = "motor: STOP   ";
              stop();
          } else if(direction == CMD_RESET){
            debug_line_2 = "System reset  ";
            delay(5000);
            ESP.restart();
          } else {
            debug_line_3 = "Command error  ";
            update_oled_display();
          }
        } 
        // ToDo: 여기에 0x52 (Diff Drive), 0x53 (Wheel Drive) 등 다른 모드 해석을 추가합니다.
        
        else {
            debug_line_3 = "Command error  ";
            update_oled_display();
        }
      } else {
        // 헤더 불일치 시: 시리얼 버퍼 정크 데이터 처리 (옵션)
        debug_line_3 = "Packet error   ";
        update_oled_display();
      }
    }
  }
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 100) {
      send_sensors();
      //print_sensor();
      prev_ms = millis();
      debug_line_3 = "Battery: " + String(loadVoltage_V);
      update_oled_display();
  }

}

void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
  debug_line_1 = "RESET          ";
  debug_line_2 = "System start...";
  debug_line_3 = "               ";
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(debug_line_1);
  display.setCursor(0, 10);
  display.print(debug_line_2);
  display.setCursor(0, 20);
  display.print(debug_line_3);
  display.display();
  delay(2000);
  debug_line_1 = "RUNNING       ";
  debug_line_2 = "motor: STOP   ";
  debug_line_3 = "Battery: " + String(loadVoltage_V);
  update_oled_display();
}

void update_oled_display() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  // 첫번째 줄
  display.setCursor(0, 0);
  display.print(debug_line_1);
  // 두번째 줄
  display.setCursor(0, 10);
  display.print(debug_line_2); 
  // 세번째 줄
  display.setCursor(0, 20);
  display.print(debug_line_3);
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

void go_forward(int speed){
  motorA.control(speed);
  motorB.control(speed);
}

void go_backward(int speed){
  motorA.control(speed);
  motorB.control(speed);
}

void turn_right(int speed){
  motorA.control(speed);
  motorB.control(speed);
}

void turn_left(int speed){
  motorA.control(speed);
  motorB.control(speed);
}

void stop(){
  motorA.stop();
  motorB.stop();
}

void send_sensors() {
    // 헤더 (0-1)
    sensor_data_packet[0] = 0xf5; 
    sensor_data_packet[1] = 0xf5; 
    
    // 가속도 (2-7)
    sensor_data_packet[2] = (ax_l >> 8) & 0xff;
    sensor_data_packet[3] = ax_l & 0xff; 
    sensor_data_packet[4] = (ay_l >> 8) & 0xff;
    sensor_data_packet[5] = ay_l & 0xff;
    sensor_data_packet[6] = (az_l >> 8) & 0xff;
    sensor_data_packet[7] = az_l & 0xff;
    
    // 자이로 (8-13)
    sensor_data_packet[8] = (gx_l >> 8) & 0xff;
    sensor_data_packet[9] = gx_l & 0xff; 
    sensor_data_packet[10] = (gy_l >> 8) & 0xff;
    sensor_data_packet[11] = gy_l & 0xff;
    sensor_data_packet[12] = (gz_l >> 8) & 0xff;
    sensor_data_packet[13] = gz_l & 0xff;
    
    // 마그네틱 (14-19)
    sensor_data_packet[14] = (mx_l >> 8) & 0xff;
    sensor_data_packet[15] = mx_l & 0xff; 
    sensor_data_packet[16] = (my_l >> 8) & 0xff;
    sensor_data_packet[17] = my_l & 0xff;
    sensor_data_packet[18] = (mz_l >> 8) & 0xff;
    sensor_data_packet[19] = mz_l & 0xff;
    
    // 엔코더 (20-27)
    sensor_data_packet[20] = (lf_encoder >> 8) & 0xff;
    sensor_data_packet[21] = lf_encoder & 0xff;
    sensor_data_packet[22] = (rf_encoder >> 8) & 0xff;
    sensor_data_packet[23] = rf_encoder & 0xff;
    sensor_data_packet[24] = (lr_encoder >> 8) & 0xff;
    sensor_data_packet[25] = lr_encoder & 0xff;
    sensor_data_packet[26] = (rr_encoder >> 8) & 0xff;
    sensor_data_packet[27] = rr_encoder & 0xff;
    
    // 체크섬 (28) (임시로 0x00)
    sensor_data_packet[28] = 0x00; 
    
    // Serial 포트로 바이너리 데이터 전송
    Serial.write(sensor_data_packet, SENSOR_PACKET_LENGTH); 

}

// 디버깅용으로 시리얼로 센서 및 엔코더 데이터 출력 
void print_sensor(){
  //Serial.print(ax_l); Serial.print(" "); Serial.print(ay_l); Serial.print(az_l); Serial.println();
  //Serial.print(gx_l); Serial.print(" "); Serial.print(gy_l); Serial.print(gz_l); Serial.println();
  //Serial.print(mx_l); Serial.print(" "); Serial.print(my_l); Serial.print(mz_l); Serial.println();
  Serial.print(lf_encoder); Serial.print(" "); Serial.print(rf_encoder); Serial.println();
}

