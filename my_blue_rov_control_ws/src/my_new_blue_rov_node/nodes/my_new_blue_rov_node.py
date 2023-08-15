#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn

def main():
    # Initialize the ROS node
    rospy.init_node('blue_rov2_communication_node', anonymous=True)
    
    # Create a publisher to send RC override commands
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    
    # Set the control values (assuming a 6-channel RC setup)
    rc_channels = OverrideRCIn()
    rc_channels.channels[0] = 1500  # Throttle
    rc_channels.channels[1] = 1500  # Roll
    rc_channels.channels[2] = 1500  # Pitch
    rc_channels.channels[3] = 1500  # Yaw
    rc_channels.channels[4] = 1000  # Auxiliary channel 1
    rc_channels.channels[5] = 1000  # Auxiliary channel 2
    
    # Send the RC override command periodically
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(rc_channels)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass