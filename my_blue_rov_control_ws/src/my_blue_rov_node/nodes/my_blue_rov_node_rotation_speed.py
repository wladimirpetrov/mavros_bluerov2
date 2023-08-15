#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn

GIMBALL_CHANNEL = 8

def send_camera_commands():
    print('Starting node...')
    rospy.init_node('blue_rov_camera_node', anonymous=True)
    rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    command = OverrideRCIn()
    command.channels[GIMBALL_CHANNEL] = 1500  # Set camera tilt command (channel 8) to neutral position

    rate = rospy.Rate(10)  # Specify the rate (in Hz) at which to send commands

    print('Node started!')

    while not rospy.is_shutdown():
        # Send camera tilt up command
        print('Tilting up...')
        command.channels[GIMBALL_CHANNEL] = 1600  # Increase the value for tilting the camera up
        rc_pub.publish(command)
        #rate.sleep()

        # Wait for a moment
        rospy.sleep(2.0)

        # Send camera tilt down command
        print('Tilting down...')
        command.channels[GIMBALL_CHANNEL] = 1400  # Decrease the value for tilting the camera down
        rc_pub.publish(command)
        #rate.sleep()

        # Wait for a moment
        rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        send_camera_commands()
        print('Done.')
    except rospy.ROSInterruptException:
        print('Interrupt')

# rostopic echo /mavros/rc/override
