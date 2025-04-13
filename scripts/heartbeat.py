#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

class GPSListener:
    def __init__(self):
        self.received_data = False
        self.co_ords = (0.0, 0.0)
        
        rospy.init_node('float_subscriber', anonymous=True)
        rospy.Subscriber('/float_topic', Float32, self.callback)
        rospy.Subscriber('/gps_data', NavSatFix, self.gps)
        
        self.rate = rospy.Rate(0.2)  # 0.2 Hz (5 seconds)
    
    def callback(self, data):
        self.received_data = True
        # rospy.loginfo("Received data: %f", data.data)

    def gps(self, data):
        self.co_ords = (data.latitude, -(data.longitude))
    
    def listener(self):
        while not rospy.is_shutdown():
            if self.received_data:
                print("I'm alive")
                print("Current co-ordinates are: ", self.co_ords)
                self.received_data = False
            else: 
                print("I'm dead")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gps_listener = GPSListener()
        gps_listener.listener()
    except rospy.ROSInterruptException:
        pass
