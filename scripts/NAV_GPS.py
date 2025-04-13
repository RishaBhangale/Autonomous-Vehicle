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
        self.current_coords = (0,0) 
        # Example coordinates
        self.desired_coords_list = [
            (19.2110575, 72.8677195)
        ]
        self.current_index = 0
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.end = False

        rospy.Subscriber('/gps_data', NavSatFix, self.gps)
        rospy.Subscriber('/Bot/imu', Imu, self.imu)

        self.cuur_lat = rospy.Publisher('/curr_lat', Float32, queue_size=1)
        self.cuur_long = rospy.Publisher('/curr_long', Float32, queue_size=1)
        self.yaw_off_curr = rospy.Publisher('/yaw_off_curr', Float32, queue_size=1)
        self.yaw_off_des = rospy.Publisher('/yaw_off_des', Float32, queue_size=1)
        self.gps_stop = rospy.Publisher('/gps_stop', Float32, queue_size=1)

    def quaternion_to_euler(self):
        t3 = +2.0 * (self.quaternion[3] * self.quaternion[2] + self.quaternion[0] * self.quaternion[1])
        t4 = +1.0 - 2.0 * (self.quaternion[1] * self.quaternion[1] + self.quaternion[2] * self.quaternion[2])
        yaw = math.atan2(t3, t4)

        lat1, lon1 = self.current_coords
        lat2, lon2 = self.desired_coords_list[self.current_index]
        current_heading = (math.degrees(yaw) + 360) % 360
        current_heading = 360 - current_heading

        desired_heading = self.calculate_bearing(lat1, lon1, lat2, lon2)
        self.yaw_off_curr.publish(current_heading)
        if(self.current_index != 0):
            self.yaw_off_des.publish(desired_heading)
        
        return current_heading, desired_heading

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing
    
    def gps(self, data):
        self.current_coords = (data.latitude, -(data.longitude))  # Fixing the negative longitude if necessary
        self.cuur_lat.publish(data.latitude)
        self.cuur_long.publish(data.longitude)
        if not self.end and self.is_at_desired_coords():
            self.current_index += 1
            if self.current_index >= len(self.desired_coords_list):
                self.end = True
                print("Reached final destination.")
                self.gps_stop.publish(10000.0001)
            else:
                print(f"Heading to next waypoint: {self.desired_coords_list[self.current_index]}")

    def is_at_desired_coords(self):
        lat1, lon1 = self.current_coords
        lat2, lon2 = self.desired_coords_list[self.current_index]
        return abs(lat1 - lat2) < 0.0000001 and abs(lon1 - lon2) < 0.0000001

    def imu(self, data):
        self.quaternion = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        ]

    def shutdown(self):
        rospy.loginfo("Shutting down GPS Navigator node.")


    def publisher(self):
        print(self.current_index)
        if(self.current_index != 0):
                self.yaw_off_des.publish(desired_heading)

if __name__ == '__main__':
    navigator = GPSNavigator()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        if not navigator.end:
            navigator.publisher()
            current_heading, desired_heading = navigator.quaternion_to_euler()
            print("CURRENT HEADING:", current_heading)
            print("DESIRED HEADING:", desired_heading)
            
        r.sleep()
