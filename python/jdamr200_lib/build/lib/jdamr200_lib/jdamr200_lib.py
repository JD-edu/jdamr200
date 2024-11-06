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

class Jdamr200(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        self.head = 0xf5
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

        self.GO_FORWARD = 1
        self.GO_BACKWARD = 2
        self.TURN_LEFT = 3
        self.TURN_RIGHT = 4
        self.STOP = 5

        self.MOVE_RUN_MODE = 0x51
        self.MOVE_WHELL_MODE = 0x52
        self.MOVE_RUN_DIFF = 0x53

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
    jdARM200 has 2 speed command protocols. 
    1. run mode: You can control AMR as followungs:
       go forward  1 
       go backward   2
       turn left   3
       turn right  4
       stop   5
    2. wheel mode: You can control each wheel respectively 
    3. Differential mode 
    ToDo: 2, 3 control mode 
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
            buf = bytearray([0xf5, 0xf5, self.MOVE_RUN_MODE, self.STOP, speed])
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

if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = Jdamr200(com)
    bot.receive_thread()
    time.sleep(1)
   
    while True:
        '''
        To debug sensor reading 
        '''
        #time.sleep(0.05)
        #bot.readSpeed()
        '''
        To debug simple motor control 
        '''
        bot.move_run_mode(bot.GO_FORWARD, 50)
        time.sleep(2)
        bot.move_run_mode(bot.GO_FORWARD, 200)
        time.sleep(2)
        bot.move_run_mode(bot.GO_BACKWARD, 50)
        time.sleep(2)
        bot.move_run_mode(bot.GO_BACKWARD, 200)
        time.sleep(2)