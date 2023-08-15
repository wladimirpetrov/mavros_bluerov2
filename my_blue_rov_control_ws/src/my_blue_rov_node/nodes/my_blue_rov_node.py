#!/usr/bin/env python

import rospy
from mavros_msgs.msg import MountControl

def tilt_camera(tilt_angle):
    mount_control_msg = MountControl()
    mount_control_msg.header.stamp = rospy.Time.now()
    mount_control_msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
    mount_control_msg.pitch = tilt_angle  # Set your desired tilt angle here
    # You can also set roll and yaw values if necessary
    # mount_control_msg.roll = 0.0
    # mount_control_msg.yaw = 0.0
    mount_control_pub.publish(mount_control_msg)

if __name__ == "__main__":
    rospy.init_node("camera_tilt_controller")
    mount_control_pub = rospy.Publisher("/mavros/mount_control/command", MountControl, queue_size=10)

    # Example: Tilt the camera by 30 degrees (You can adjust the angle as needed)
    tilt_angle_degrees = 45.0
    tilt_angle_radians = tilt_angle_degrees * 3.14159 / 180.0

    try:
        tilt_camera(tilt_angle_radians)
        rospy.loginfo("Camera tilted by {} degrees.".format(tilt_angle_degrees))
    except rospy.ROSInterruptException:
        rospy.logerr("Error occurred while tilting the camera.")
