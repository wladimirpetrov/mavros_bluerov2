#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn

def control_thrusters():
    rospy.init_node('bluerov_thruster')
    rate = rospy.Rate(1)  # 10 Hz update rate

    # Create a publisher for the RC override topic
    rc_pub = rospy.Publisher('/mavros/rc/out', OverrideRCIn, queue_size=10)

    while not rospy.is_shutdown():
        # Create an instance of the OverrideRCIn message
        rc_msg = OverrideRCIn()

        # Mimic joystick inputs for pitch, roll, throttle, yaw, forward, and lateral movements
        # Adjust these values to rotate the thrusters as desired
        rc_msg.channels = [1500, 1500, 1500, 1600, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

        # Publish the RC message
        rc_pub.publish(rc_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        control_thrusters()
    except rospy.ROSInterruptException:
        pass
