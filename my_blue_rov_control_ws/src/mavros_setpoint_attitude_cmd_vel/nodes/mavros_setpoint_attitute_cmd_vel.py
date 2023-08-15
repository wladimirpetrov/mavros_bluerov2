#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

def send_attitude_command():
    rospy.init_node('blue_rov_attitude_command', anonymous=True)
    att_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=10)
    rate = rospy.Rate(10)  # Publish at a rate of 10 Hz

    # Create a TwistStamped message for attitude command
    command = TwistStamped()
    command.header.stamp = rospy.Time.now()
    command.twist.angular.x = 0.0  # Roll rate (set to 0 for no roll movement)
    command.twist.angular.y = 0.0  # Pitch rate (set to 0 for no pitch movement)
    command.twist.angular.z = 0.5  # Yaw rate (positive for clockwise rotation, negative for counter-clockwise)

    while not rospy.is_shutdown():
        att_pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_attitude_command()
    except rospy.ROSInterruptException:
        pass

# rostopic echo /mavros/setpoint_attitude/cmd_vel