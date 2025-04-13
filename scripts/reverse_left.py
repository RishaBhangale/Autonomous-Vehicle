#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import numpy

# Create a publisher to send control commands
# pub_acc = rospy.Publisher('/robot_acc_topic', Float32, queue_size=10) # Replace 'YourControlMessageType' with your actual control message type 
pub_lft = rospy.Publisher('/robot_left_rev_topic', Float32, queue_size=10)

def controller_callback(data):
    L1 = data.buttons[4]
    left_rev = L1

#     Left_stick = data.axes[0]
#     Steer = Left_stick

#     pub_acc.publish(acceleration)
    pub_lft.publish(left_rev)
#     rospy.loginfo(acceleration)
    rospy.loginfo(left_rev)

def controller_listener():
    rospy.init_node('controller_left_rev')
    rospy.Subscriber('/joy', Joy, controller_callback)  # Subscribe to the controller topic
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_listener()
    except rospy.ROSInterruptException:
        pass