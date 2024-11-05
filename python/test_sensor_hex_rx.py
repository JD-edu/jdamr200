import serial 
import struct

ax = 0.0
ay = 0.0
az = 0.0
wx = 0.0
wy = 0.0
wz = 0.0
mx = 0.0
my = 0.0
mz = 0.0
lf_encoder = 0
rf_encoder = 0
lr_encoder = 0
rr_encoder = 0 

port="/dev/ttyACM0"
ser = serial.Serial(port, 115200)
ser.flushInput()

while True:
    
    head = bytearray(ser.read())[0]
    if head == 0xf5:
        head = bytearray(ser.read())[0]
        length = bytearray(ser.read())[0]
        if head == 0xf5:
            byte_array = ser.read(2)
            ax = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            ay = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            az = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            gx = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            gy = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            gz = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            mx = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            my = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            mz = struct.unpack('>h', byte_array)[0]/1000
            byte_array = ser.read(2)
            lf_encoder = struct.unpack('>h', byte_array)[0]
            byte_array = ser.read(2)
            lr_encoder = struct.unpack('>h', byte_array)[0]
            byte_array = ser.read(2)
            rf_encoder = struct.unpack('>h', byte_array)[0]
            byte_array = ser.read(2)
            rr_encoder = struct.unpack('>h', byte_array)[0]
            #print(lf_encoder, lr_encoder)
            print(ax, ay, az)
            #print(gx, gy, gz)
            #print(mx, my, mz)