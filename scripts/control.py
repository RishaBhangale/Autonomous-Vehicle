#!/usr/bin/env python3
import rospy
from evdev import InputDevice, list_devices
from subprocess import Popen, PIPE, STDOUT

def is_controller_connected():
    devices = [InputDevice(path) for path in list_devices()]
    for dev in devices:
        if dev.name == 'Microsoft X-Box 360 pad':
            return True
    return False

def check_controller_status(event):
    controller_connected = is_controller_connected()
    rospy.loginfo("Acknowledgement Received")

if __name__ == "__main__":
    rospy.init_node('controller_monitor')
    controller_connected = is_controller_connected()

    if controller_connected == True:
        # Run node A
        rospy.loginfo("Launching Node Controller")
        Popen(['rosrun', 'joy', 'joy_node'])
        Popen(['rosrun', 'igvc', 'left_controller.py'])
        Popen(['rosrun', 'igvc', 'right_controller.py'])
        Popen(['rosrun', 'igvc', 'reverse_left.py'])
        Popen(['rosrun', 'igvc', 'reverse_right.py'])
        # Popen(['rosrun', 'igvc', 'e-kill.py'])
    else:
        # Run node B    
        rospy.loginfo("Launching Node Self-Driving")
        Popen(['rosrun', 'igvc', 'OBJ.py'])
        # Popen(['rosrun', 'igvc', 'NAV_GPS.py'])
        

    rospy.spin()