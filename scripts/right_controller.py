#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import numpy

# Create a publisher to send control commands
pub_rht = rospy.Publisher('/robot_right_topic', Float32, queue_size=10) # Replace 'YourControlMessageType' with your actual control message type 
#pub_rev = rospy.Publisher('/robot_rev_topic', Float32, queue_size=10)
# pub_steer = rospy.Publisher('/robot_steer_topic', Float32, queue_size=10)

def controller_callback(data):
    R2 = data.axes[5]
    right = R2
    # A = data.buttons[0]
    # rev = A
    # Left_stick = data.axes[0]                                                      
    # Steer = Left_stick
    # pub_rev.publish(rev)
    pub_rht.publish(right)
    # pub_steer.publish(Steer)
    rospy.loginfo(right)
    # rospy.loginfo(Steer)

def controller_listener():
    rospy.init_node('controller_right')
    rospy.Subscriber('/joy', Joy, controller_callback)  # Subscribe to the controller topic
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_listener()
    except rospy.ROSInterruptException:
        pass