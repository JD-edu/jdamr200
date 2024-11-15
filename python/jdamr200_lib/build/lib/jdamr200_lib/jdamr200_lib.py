'''
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
'''

import serial
import threading 
import struct
import time 
import threading 
import math

class Jdamr200(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        # packet header 
        self.head = 0xf5
        # IMU, encoder low level data 
        self.ax = 0.0 
        self.ay = 0.0
        self.az = 0.0 
        self.gx = 0.0
        self.gy = 0.0 
        self.gz = 0.0 
        self.mx = 0.0
        self.my = 0.0 
        self.mz = 0.0 
        self.lf_encoder = 0
        self.rf_encoder = 0
        self.lr_encoder = 0
        self.rr_encoder = 0

        # data for odmetry claculation 
        self.wheel_radius = 0.05  # wheel diameter 5cm 
        self.wheel_base = 0.30    # distance between wheels (m)
        self.encoder_resolution = 360  # encoder resolution (number of pulse)
        self.previous_left_encoder = 0
        self.previous_right_encoder = 0
        
        # 초기 포지션 (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.GO_FORWARD = 1
        self.GO_BACKWARD = 2
        self.TURN_LEFT = 3
        self.TURN_RIGHT = 4
        self.STOP = 5

        self.MOVE_RUN_MODE = 0x51
        self.MOVE_RUN_DIFF = 0x52
        self.MOVE_WHEEL_MODE = 0x53

    def readSpeed(self):
        buf_header = bytearray(1)
        buf = bytearray(27)
        header_found = False
        count = 0

        try:
            while not header_found:
                count += 1
                ret = self.ser.readinto(buf_header)
                if ret != 1:
                    continue
                if buf_header[0] != 0xf5:
                    continue
                
                header_2_found = False
                while not header_2_found:
                    ret = self.ser.readinto(buf_header)
                    if ret != 1:
                        continue
                    if buf_header[0] != 0xf5:
                        continue
                    header_2_found = True
                header_found = True
   

        
            # 데이터 읽기
            ret = self.ser.readinto(buf)
            if ret == 27:
                # accel
                ax_array = buf[0:2]
                self.ax = struct.unpack('>h', ax_array)[0]/1000
                ay_array = buf[2:4]
                self.ay = struct.unpack('>h', ay_array)[0]/1000
                az_array = buf[4:6]
                self.az = struct.unpack('>h', az_array)[0]/1000
                # Gyro
                gx_array = buf[6:8]
                self.gx = struct.unpack('>h', gx_array)[0]/1000
                gy_array = buf[8:10]
                self.gy = struct.unpack('>h', gy_array)[0]/1000
                gz_array = buf[10:12]
                self.gz = struct.unpack('>h', gz_array)[0]/1000
                # magnetic 
                mx_array = buf[12:14]
                self.mx = struct.unpack('>h', mx_array)[0]/1000
                my_array = buf[14:16]
                self.my = struct.unpack('>h', my_array)[0]/1000
                mz_array = buf[16:18]
                self.mz = struct.unpack('>h', mz_array)[0]/1000
                # Odmetry 
                lf_array = buf[18:20]
                self.lf_encoder = struct.unpack('>h', lf_array)[0]
                rf_array = buf[20:22]
                self.rf_encoder = struct.unpack('>h', rf_array)[0]
                lr_array = buf[22:24]
                self.lr_encoder = struct.unpack('>h', lr_array)[0]
                rr_array = buf[24:26]
                self.rr_encoder = struct.unpack('>h', rr_array)[0]


                # for debugging 
                #print(self.ax, self.ay, self.az)
                #print(self.gx, self.gy, self.gz)
                #print(self.mx, self.my, self.mz)
                print(self.lf_encoder, self.lr_encoder, self.rf_encoder, self.rr_encoder)
            else:
                return 
        except:
            print("serial read error")
            return
    '''
    jdARM200 has 3 speed command protocols. 
    1. run mode: You can control AMR as followungs:
       go forward  1 
       go backward   2
       turn left   3
       turn right  4
       stop   5
    2. wheel mode: You can control each wheel respectively 
    3. Differential mode 
    '''
    def move_run_mode(self ,run_mode, speed):
        if run_mode == 1:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.GO_FORWARD, speed])
        elif run_mode == 2:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.GO_BACKWARD, speed])
        elif run_mode == 3:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.TURN_LEFT, speed])
        elif run_mode == 4:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.TURN_RIGHT, speed])
        elif run_mode == 5:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.STOP, speed])
        else:
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.STOP, 0])
        # send packet to serila port 
        print(buf)
        self.ser.write(buf)

    '''
    In differential drive mode, we provide steering angle and speed.
    comamnd code is 0x52, angle is between 150 an 30.
    (0xf5, 0xf5, 0x52, angle, speed)
    '''
    def move_diff_mode(self, angle, speed):
        if angle > 150:
            angle = 150 
        elif angle < 30:
            angle = 30
        buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_DIFF, angle, speed])
        # send packet to serila port 
        print(buf)
        self.ser.write(buf)
    
    '''
    In wheel mode, we can control each 4 wheel respectively. 
    command code is 0x53
    (0xf5, 0xf5, 0x53, wheel number, speed)
    lf: 1 lr: 2 rf: 3 rr: 4 
    '''
    def move_wheel_mode(self, wheel_no, speed):
        buf = bytearray([0xf5, 0xf5,  self.MOVE_WHEEL_MODE, wheel_no, speed])
        # send packet to serila port 
        print(buf)
        self.ser.write(buf)


    def receive_thread(self):
        try:
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    def receive_data(self):     
        self.ser.flushInput()
        while True:
            self.readSpeed()
            time.sleep(0.05)

    def update_odometry(self):
        self.lf_encoder += 1
        self.rf_encoder += 1
        # update encoder value 
        current_left_encoder = self.lf_encoder  
        current_right_encoder = self.rf_encoder 
        
        # get encoder tick delta
        delta_left = current_left_encoder - self.previous_left_encoder
        delta_right = current_right_encoder - self.previous_right_encoder
        
        # calculate turn angle from encoder tick 
        left_distance = (2 * math.pi * self.wheel_radius) * (delta_left / self.encoder_resolution)
        right_distance = (2 * math.pi * self.wheel_radius) * (delta_right / self.encoder_resolution)
        
        # calculate average moving distance delta and angular speed 
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # update position 
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta
        
        # 이전 엔코더 값 갱신
        self.previous_left_encoder = current_left_encoder
        self.previous_right_encoder = current_right_encoder
        return self.x, self.y, self.theta

if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = Jdamr200(com)
    bot.receive_thread()
    time.sleep(2)
   
    while True:
        '''
        To debug sensor reading 
        '''
        #time.sleep(0.05)
        #bot.readSpeed()
        '''
        To debug simple motor control run_mode 
        '''
        bot.move_run_mode(bot.GO_FORWARD, 100)
        time.sleep(2)
        bot.move_run_mode(bot.GO_BACKWARD, 100)
        time.sleep(2)
        bot.move_run_mode(bot.TURN_LEFT, 100)
        time.sleep(2)
        bot.move_run_mode(bot.TURN_RIGHT, 100)
        time.sleep(2)
        bot.move_run_mode(bot.STOP, 0)
        time.sleep(2)
        '''
        To debug simple differential mode test 
        '''
        #bot.move_diff_mode(30, 100)
        #time.sleep(2)
        #bot.move_diff_mode(150, 100)
        #time.sleep(2)

        '''
        To debug simple wheel mode test 
        '''
        #bot.move_wheel_mode(1, 100)
        #time.sleep(1)
        #bot.move_wheel_mode(1, 0)
        #time.sleep(1)
        #bot.move_wheel_mode(2, 100)
        #time.sleep(1)
        #bot.move_wheel_mode(2, 0)
        #time.sleep(1)
        #bot.move_wheel_mode(3, 100)
        #time.sleep(1)
        #bot.move_wheel_mode(3, 0)
        #time.sleep(1)
        #bot.move_wheel_mode(4, 100)
        #time.sleep(1)
        #bot.move_wheel_mode(4, 0)
        #time.sleep(1)


        
