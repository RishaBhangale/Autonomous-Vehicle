#!/usr/bin/env python3
import numpy as np
import math
import tf
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class GPSNavigator:

    def __init__(self):
        rospy.init_node('GPS_NAV')
        print("MAIN CONTROLLER START")
        rospy.on_shutdown(self.shutdown)
        self.threshhold = [42.607232,-83.251265]
        self.current_coords = (42.607256, -83.250795) # Example coordinates
        self.desired_coords = (42.668190,-83.217473)  # Example coordinates
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.end=False 
        # self.imu_data = [0,0,0,0]

        rospy.Subscriber('/gps_data', NavSatFix, self.gps)
        rospy.Subscriber('/Bot/imu', Imu, self.imu)

        # self.lin_off = rospy.Publisher('/lin_off', Float32, queue_size=1)
        self.yaw_off_curr = rospy.Publisher('/yaw_off_curr', Float32, queue_size=1)
        self.yaw_off_des = rospy.Publisher('/yaw_off_des', Float32, queue_size=1)


    # Function to convert quaternion to Euler angles
    def quaternion_to_euler(self):
        # # Quaternion to roll (x-axis rotation)
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll = math.atan2(t0, t1)
        
        # # Quaternion to pitch (y-axis rotation)
        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch = math.asin(t2)
        
        # Quaternion to yaw (z-axis rotation)
        t3 = +2.0 * (self.quaternion[3] * self.quaternion[2] + self.quaternion[0] * self.quaternion[1])
        t4 = +1.0 - 2.0 * (self.quaternion[1] * self.quaternion[1] + self.quaternion[2] * self.quaternion[2])
        yaw = math.atan2(t3, t4)

        lat1, lon1 = self.current_coords
        lat2, lon2 = self.desired_coords
        x, y, z, w = self.quaternion

        # Convert quaternion to Euler angles (specifically yaw)
        # roll, pitch, yaw = self.quaternion_to_euler(w, x, y, z)
        current_heading = (math.degrees(yaw) + 360) % 360
        current_heading = 360 - current_heading

        # Calculate the bearing to the desired coordinates
        desired_heading = self.heavyside_bearing(lat1, lon1, lat2, lon2)
        self.yaw_off_curr.publish(current_heading)
        self.yaw_off_des.publish(desired_heading)
        
        return current_heading,desired_heading

    # Function to calculate bearing between two coordinates
    def heavyside_bearing(self,lat1,lon1,lat2,lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        # lon2 = math.radians(self.desired_coords[1])

        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    def gps(self, data):
        self.current_coords = (data.latitude,-(data.longitude))
        
    #     print(self.current_coords)
    #     if(self.end == False):
    #         if (self.current_coords[0]>=self.current_gps_point[0]-self.threshhold[0] and self.current_coords[0]<=self.current_gps_point[0]+self.threshhold[0]) and (self.current_coords[1]>=self.current_gps_point[1]-self.threshhold[1] and self.current_coords[1]<=self.current_gps_point[1]+self.threshhold[1]):
	# 			#pass
    #             print('in')

    def imu(self, data):
        self.quaternion[0] = data.orientation.x
        self.quaternion[1] = data.orientation.y
        self.quaternion[2] = data.orientation.z
        self.quaternion[3] = data.orientation.w

    def shutdown(self):
        rospy.loginfo("Shutting down GPS Navigator node.")

if __name__ == '__main__':
    navigator = GPSNavigator()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        current_heading, desired_heading = navigator.quaternion_to_euler()
        # navigator.pidss()
        print("CURRENT HEADING")
        print(current_heading)
        print("DESIRED HEADING")
        print(desired_heading)
    
        r.sleep()


    #42.60727 -83.25084  