#!/usr/bin/env python3
import time
import serial
import rospy
from sensor_msgs.msg import NavSatFix
gps = serial.Serial('/dev/ttyACM1', 9600, timeout=1) 
# hz = bytearray([0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12])
# gps.write(hz) 
# opening COM port
gps.flushInput()                    # clears the serial port so there is no data overlaping
gps_bytes = gps.readline()
rospy.init_node('gps_serial', anonymous=True)
pub = rospy.Publisher('gps_data',NavSatFix, queue_size=10)
gps.flush()
msg=NavSatFix()
#print("Hi")
#print(gps_bytes)
while gps_bytes and not rospy.is_shutdown():
    line = gps.readline()
    # print(line)
    line = line.decode('utf-8')
    #print("...")
    # print(line)
    if line.startswith("$GNGGA"):
        time, LAT, dir1, LONG, dir2 = line.split(",")[1:6]
        gps.flushInput()
        #print(LAT,LONG
        DDlat=int(float(LAT)/100)
        SSlat=float(LAT)-DDlat*100
        Latdec=DDlat+SSlat/60

        DDlong=int(float(LONG)/100)
        SSlong=float(LONG)-DDlong*100
        Longdec=DDlong+SSlong/60
        #print(Latdec,Longdec)
        msg.latitude = Latdec
        msg.longitude = Longdec
        print("Hell0")
        print(msg)
        
        pub.publish(msg)
    else:
        time, pos, LAT, dir1, LONG, dir2, speed ,deg = 0,0,0,0,0,0,0,0
        # print('error')
        msg.latitude = LAT
        msg.longitude = LONG
        # msg.altitude = speed
        print("BYE")
        pub.publish(msg)