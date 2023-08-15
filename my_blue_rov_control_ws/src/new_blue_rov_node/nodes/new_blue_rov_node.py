#!/usr/bin/env python

import rospy
from mavros_msgs.msg import ActuatorControl

def send_actuator_control():
    rospy.init_node('blue_rov_actuator_node', anonymous=True)
    actuator_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

    actuator_msg = ActuatorControl()
    actuator_msg.group_mix = 0
    actuator_msg.controls = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Set the desired control values

    rate = rospy.Rate(10)  # Specify the rate (in Hz) at which to send the command

    while not rospy.is_shutdown():
        actuator_pub.publish(actuator_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_actuator_control()
    except rospy.ROSInterruptException:
        pass
