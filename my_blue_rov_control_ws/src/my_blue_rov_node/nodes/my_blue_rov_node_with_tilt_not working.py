#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandLong
from pymavlink import mavutil

GIMBAL_CHANNEL = 8

def configure_mount_controls():
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        mount_configure = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        mount_configure(broadcast=False, command=400, confirmation=0, param1=0, param2=0, param3=0, param4=0, param5=0, param6=-3000, param7=0)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def send_camera_commands():
    print('Starting node...')
    rospy.init_node('blue_rov_camera_node', anonymous=True)
    rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    command = OverrideRCIn()
    command.channels[GIMBAL_CHANNEL] = 1500  # Set camera tilt command (channel 8) to neutral position

    rate = rospy.Rate(10)  # Specify the rate (in Hz) at which to send commands

    print('Node started!')

    while not rospy.is_shutdown():
        # Send camera tilt up command
        print('Tilting up...')
        command.channels[GIMBAL_CHANNEL] = 1600  # Increase the value for tilting the camera up
        rc_pub.publish(command)

        # Wait for a moment
        rospy.sleep(2.0)

        # Send camera tilt down command
        print('Tilting down...')
        command.channels[GIMBAL_CHANNEL] = 1400  # Decrease the value for tilting the camera down
        rc_pub.publish(command)

        # Wait for a moment
        rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        configure_mount_controls()
        send_camera_commands()
        print('Done.')
    except rospy.ROSInterruptException:
        print('Interrupt')
