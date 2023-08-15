#!/usr/bin/env python

import rospy
from mavros_msgs.msg import CommandLong

def send_camera_tilt_command():
    rospy.init_node('blue_rov_camera_node', anonymous=True)
    command_pub = rospy.Publisher('/mavros/cmd/command', CommandLong, queue_size=10)

    command = CommandLong()
    command.command = 2000  # MAV_CMD_DO_MOUNT_CONTROL
    command.param1 = 0 # MAV_MOUNT_MODE_RETRACT
    command.param2 = 0 # Pitch angle (in degrees)

    rate = rospy.Rate(1)  # Specify the rate (in Hz) at which to send commands

    while not rospy.is_shutdown():
        # Tilt the camera to the top
        command.param2 = -45  # Set the desired pitch angle (in degrees)
        command_pub.publish(command)
        rate.sleep()

        # Wait for a moment
        rospy.sleep(2.0)

        # Tilt the camera to the bottom
        command.param2 = 45   # Set the desired pitch angle (in degrees)
        command_pub.publish(command)
        rate.sleep()

        # Wait for a moment
        rospy.sleep(2.0)

if __name__ == 'main':
    try:
        send_camera_tilt_command()
    except rospy.ROSInterruptException:
        pass