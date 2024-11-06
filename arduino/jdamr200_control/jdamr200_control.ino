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

// install Encoder library 
// https://github.com/PaulStoffregen/Encoder
#include<Encoder.h>
#include "tb6612_motor.h"

// install following library 
// https://github.com/hideakitai/MPU9250
#include "MPU9250.h"

// motor class object
// Moebius Mega2560 motor sheild 
// code reference 
// https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/MecanumRobotPS2Control.ino
// schematic 
// https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560/blob/master/mega2560__2024-7%EF%BC%88update%EF%BC%89.pdf

SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 35, 34); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 37, 36); // <- NOTE: Motor Dir pins reversed for opposite operation
/*
 In github, motor_LR has  3, 49, true,  9, 42, 43. but some board should be set  3, 49, true,  6, 42, 43 
*/
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  6, 42, 43); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, false, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation

void go_forward(int speed){
    motorLF->speed(speed); 
    motorLR->speed(speed); 
    motorRF->speed(speed);
    motorRR->speed(speed);
}

void go_backward(int speed){
    motorLF->speed(-speed); 
    motorLR->speed(-speed); 
    motorRF->speed(-speed);
    motorRR->speed(-speed);
}

void turn_right(int speed){
    motorLF->speed(speed); 
    motorLR->speed(speed); 
    motorRF->speed(-speed);
    motorRR->speed(-speed);
}

void turn_left(int speed){
    motorLF->speed(-speed); 
    motorLR->speed(-speed); 
    motorRF->speed(speed);
    motorRR->speed(speed);
}

void go_side_right(int speed){
    motorLF->speed(speed); 
    motorLR->speed(-speed); 
    motorRF->speed(-speed);
    motorRR->speed(speed);
}

void go_side_left(int speed){
    motorLF->speed(-speed); 
    motorLR->speed(speed); 
    motorRF->speed(speed);
    motorRR->speed(-speed);
}

void stop(){
    motorLF->hardStop();
    motorRF->hardStop();
    motorLR->hardStop(); 
    motorRR->hardStop();
}

MPU9250 mpu;

// Encoder variable 
int lf_encoder = 0;
int rf_encoder = 0;
int lr_encoder = 0;
int rr_encoder = 0;

// incoming command buffer 
byte cmd_buf[5];

void setup(){
    String uga = "hello"; 
    Serial.begin(115200);
    Serial2.begin(115200);  //Mega2560 16(TXD2) 17(RXD2) pin, for debugging 
    // I2C start 
    Wire.begin();
    delay(2000);
    // MPu9250 init 
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

void loop(){
    // We can control motor directly throgh serial2. 
    if(Serial2.available() > 0){
        char c = Serial2.read();
        Serial2.println(c);
        if(c == 'w'){
            go_forward(50);
        }else if(c == 's'){
            go_backward(50);
        }else if(c == 'a'){
            turn_left(50); 
        }else if(c == 'd'){
            turn_right(50);
        }else if(c == 'p'){
            stop();
        }else{
            stop();
        }
    }

    /*
    Motor Control 
    - We have 3 motor control mode 
      - run mode 
      - wheel mode 
      - differential mode 
    - run mode: controlling motor go/back/left turn/right turn
    (0xf5, 0xf5, 0x51, direction, speed)
    - ToDo: 2, 3 control mode  
    */  
    
    if(Serial.available() > 0){
        // motor control packets are 5 gytes length. we get motor control packats if available. 
        Serial.readBytes(cmd_buf, 5);
        if(cmd_buf[0]== 0xf5){          // header 1 
            if(cmd_buf[1] == 0xf5){         // header 2 
                if(cmd_buf[2] == 0x51){         // run mode? 
                    int speed = (int)cmd_buf[4];    // get speed 
                    if(cmd_buf[3] == 1){
                        go_forward(speed);          // go forward 
                    }else if(cmd_buf[3] == 2){
                        go_backward(speed);         // go backward 
                    }
                }else if(cmd_buf[2] == 0x52){
                    int speed = (int)cmd_buf[4];
                    int angle = (int)cmd_buf[3];
                    //Serial2.print(angle);
                    //Serial2.println(speed);
                    diff_drive(angle, speed);

                }else if(cmd_buf[2] == 0x53){
                    int speed = (int)cmd_buf[4];
                    int motor_no = (int)cmd_buf[3];
                    wheel_drive(motor_no, speed);

                }
            }
        }
    }
   
    // read MPU9250 
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        // We send packets every 50ms, If communication not work, use timer interrpt 
        if (millis() > prev_ms + 50) {
            // send sensor data as binary to ROS2 using serial 
            send_sensors();
            // send sensor data for debugging using serial2
            //print_gyro();
            //print_mag();
            //print_roll_pitch_yaw();
            //print_accel();
            //print_encoder();
            prev_ms = millis();
        }
    }

    // read encoder
    lf_encoder = motorLF->getEncoderPosition();
    rf_encoder = motorRF->getEncoderPosition();
    lr_encoder = motorLR->getEncoderPosition();
    rr_encoder = motorRR->getEncoderPosition();

    

}

void diff_drive(int angle, int speed){
    int angle_norm = map(angle, 30, 150, -5, 5);
    //Serial2.println(angle_norm);
    float angle_norm_f = (float)angle_norm/10;
    //Serial2.println(angle_norm_f);
    float leftSpeed = speed-(speed*(angle_norm_f));
    float rightSpeed = speed +(speed*(angle_norm_f));
    //Serial2.print(leftSpeed);
    //Serial2.print(' ');
    //Serial2.println(rightSpeed);
    motorLF->speed(leftSpeed); 
    motorLR->speed(leftSpeed); 
    motorRF->speed(rightSpeed);
    motorRR->speed(rightSpeed);
}

void wheel_drive(int wheel_no, int speed){
    switch(wheel_no){
        case 1:
            //Serial2.println("motor 1");
            motorLF->speed(speed); 
            motorRF->hardStop();
            motorLR->hardStop(); 
            motorRR->hardStop();
            break;
        case 2:
            //Serial2.println("motor 2");
            motorLF->hardStop();
            motorRF->speed(speed); 
            motorLR->hardStop(); 
            motorRR->hardStop();
            break;
        case 3:
            //Serial2.println("motor 3");
            motorLF->hardStop();
            motorRF->hardStop();
            motorLR->speed(speed);
            motorRR->hardStop();
            break;
        case 4:
            //Serial2.println("motor 4");
            motorLF->hardStop();
            motorRF->hardStop();
            motorLR->hardStop(); 
            motorRR->speed(speed);
            break;
        default:
            stop();
            break;
    }
}

void send_sensors(){
    /*
     In this function we send following sensor values. 
     - accelerometer 
     - gyro sensor 
     - magnetic sensor 
     - 4 wheel encoder 
     We do not perform sensor fusion in embedded side. we need to do on ROS2 side. 
    */
    byte data[29] = {0,};       // sensor data packet is totally 28 bytes 
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    int ax_l = int(ax * 1000);  // row accel value is between -1 ~ 1 
    int ay_l = int(ay * 1000);  // for serial sendig we multiplay it with 1000.
    int az_l = int(az * 1000);  // row accel is float and converted vlaue is intger 2 bytes 
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();
    int gx_l = int(gx * 1000);
    int gy_l = int(gy * 1000);
    int gz_l = int(gz * 1000);
    float mx = mpu.getMagX();
    float my = mpu.getMagY();
    float mz = mpu.getMagZ();
    int mx_l = int(mx * 1000);
    int my_l = int(my * 1000);
    int mz_l = int(mz * 1000);
    int length = 29; 
   
    data[0] = 0xf5;                 // first header 0xf5 We use dual header bytes for safety.
    data[1] = 0xf5;                 // second header 0xf5 
    data[2] = (ax_l >> 8) & 0xff;   // accel
    data[3] = ax_l & 0xff; 
    data[4] = (ay_l >> 8) & 0xff;
    data[5] = ay_l & 0xff;
    data[6] = (az_l >> 8) & 0xff;
    data[7] = az_l & 0xff;
    data[8] = (gx_l >> 8) & 0xff;   // gyro 
    data[9] = gx_l & 0xff; 
    data[10] = (gy_l >> 8) & 0xff;
    data[11] = gy_l & 0xff;
    data[12] = (gz_l >> 8) & 0xff;
    data[13] = gz_l & 0xff;
    data[14] = (mx_l >> 8) & 0xff;  // magnetic
    data[15] = mx_l & 0xff; 
    data[16] = (my_l >> 8) & 0xff;
    data[17] = my_l & 0xff;
    data[18] = (mz_l >> 8) & 0xff;
    data[19] = mz_l & 0xff;
    data[20] = (lf_encoder >> 8) & 0xff;  // encoder 
    data[21] = lf_encoder & 0xff;
    data[22] = (rf_encoder >> 8) & 0xff;
    data[23] = rf_encoder & 0xff;
    data[24] = (lr_encoder >> 8) & 0xff;
    data[25] = lr_encoder & 0xff;
    data[26] = (rr_encoder >> 8) & 0xff;
    data[27] = rr_encoder & 0xff;
    data[28] = 0x00;  // checksum ToDo: implementing checksum    
    Serial.write(data, 29);  // Sending command packets 
}

/*
Debugging functions 
We cna use serial2 as debugging serial port. Seriali used for communication with ROS.
*/
void print_roll_pitch_yaw() {
    Serial2.print("Yaw, Pitch, Roll: ");
    Serial2.print(mpu.getYaw(), 2);
    Serial2.print(", ");
    Serial2.print(mpu.getPitch(), 2);
    Serial2.print(", ");
    Serial2.println(mpu.getRoll(), 2);
}

void print_accel() {
    Serial2.print("Accel: ");
    Serial2.print(mpu.getAccX(), 2);
    Serial2.print(", ");
    Serial2.print(mpu.getAccY(), 2);
    Serial2.print(", ");
    Serial2.println(mpu.getAccZ(), 2);
}

void print_gyro() {
    Serial2.print("Gyro: ");
    Serial2.print(mpu.getGyroX(), 2);
    Serial2.print(", ");
    Serial2.print(mpu.getGyroY(), 2);
    Serial2.print(", ");
    Serial2.println(mpu.getGyroZ(), 2);
}

void print_mag() {
    Serial2.print("Mag: ");
    Serial2.print(mpu.getMagX(), 2);
    Serial2.print(", ");
    Serial2.print(mpu.getMagY(), 2);
    Serial2.print(", ");
    Serial2.println(mpu.getMagZ(), 2);
}

void print_encoder(){
    Serial2.print("Encoder: ");
    Serial2.print(lf_encoder);
    Serial2.print(", ");
    Serial2.print(rf_encoder);
    Serial2.print(", ");
    Serial2.print(lr_encoder);
    Serial2.print(", ");
    Serial2.print(rr_encoder);
    Serial2.println("   ");

}
