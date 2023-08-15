#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn

def control_thrusters():
    rospy.init_node('bluerov_thruster_control')
    rate = rospy.Rate(10)  # 10 Hz update rate

    # Create a publisher for the RC override topic
    rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    while not rospy.is_shutdown():
        # Create an instance of the OverrideRCIn message
        rc_msg = OverrideRCIn()

        # Set the desired PWM values for pitch, roll, and throttle
        rc_msg.channels = [1500, 1500, 1500, 1900, 1800, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

        # Override values for pitch (channel 1), roll (channel 2), and throttle (channel 3)
        rc_msg.channels[0] = 2000  # Set pitch to a higher value (Move forward)
        rc_msg.channels[1] = 1100  # Set roll to a lower value (Move left)
        rc_msg.channels[2] = 1900  # Set throttle to a higher value (Move up)

        # Publish the RC message
        rc_pub.publish(rc_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        control_thrusters()
    except rospy.ROSInterruptException:
        pass
