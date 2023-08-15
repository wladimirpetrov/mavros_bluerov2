#!/usr/bin/env python3

import rospy
import subprocess
import os
import numpy as np
from std_msgs.msg import Float64

class AttitudeControl:
    def __init__(self):
        rospy.init_node("attitude_control")
        self.z_ref_sub = rospy.Subscriber("/z_ref_msg", Float64, self.z_ref_callback)
        self.psi_ref_sub = rospy.Subscriber("/psi_ref_msg", Float64, self.psi_ref_callback)
        self.x_des_sub = rospy.Subscriber("/x_des_msg", Float64, self.x_des_callback)
        self.y_des_sub = rospy.Subscriber("/y_des_msg", Float64, self.y_des_callback)
        
        self.Ts = 0.05

        self.z_ref = 0.0
        self.psi_ref = 0.0
        self.x_des = 0.0
        self.y_des = 0.0

        self.relative_alt = 0.0

        self.zerr_prev = 0.0
        self.zerrint_prev = 0.0

        # Read PID gains and additional variables from the ROS parameter server
        self.kp_elevator = rospy.get_param('~kp_elevator', 2.0)
        self.ki_elevator = rospy.get_param('~ki_elevator', 0.5)
        self.kd_elevator = rospy.get_param('~kd_elevator', 0.1)

        self.mavlink_requesting_position_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_requesting_position.py")
#        self.mavlink_override_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_override.py")

        self.launch_mavlink_requesting_position()

    def z_ref_callback(self, msg):
        self.z_ref = msg.data

    def psi_ref_callback(self, msg):
        self.psi_ref = msg.data

    def x_des_callback(self, msg):
        self.x_des = msg.data

    def y_des_callback(self, msg):
        self.y_des = msg.data
    
    def heartbeat_callback(self, msg):
        self.relative_alt = msg.data
        z_error = self.z_ref - self.relative_alt
        elevator = self.PID(z_error, self.kp_elevator, self.ki_elevator, self.kd_elevator, self.zerr_prev, self.zerrint_prev)
        elevator = self.limit_elevator(elevator)
        print("Received Relative Altitude:", self.relative_alt)
        print("Desired Altitude:", self.z_ref)
        print("Elevator Control Signal:", elevator)
        self.zerr_prev = z_error
        self.zerrint_prev = self.zerrint_prev + z_error * self.Ts
        self.launch_mavlink_override(elevator)

    def PID(self, error, kp, ki, kd, prev_err, integral):
        integral = integral + error * self.Ts
        # Limit the integral term to prevent windup
        integral = max(min(integral, 5), -5)
        derivative = (error - prev_err) / self.Ts
        output = kp * error + ki * integral + kd * derivative
        return output

    def limit_elevator(self, value):
        if value >= 20:
            return 20
        elif value <= -20:
            return -20
        else:
            return value

    def launch_mavlink_requesting_position(self):
        subprocess.Popen(["python3", self.mavlink_requesting_position_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

#    def launch_mavlink_override(self, n_rpm):
#        mavlink_example_override_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_override.py")
#        self.mavlink_override_process = subprocess.Popen(["python3", mavlink_example_override_path, str(n_rpm)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def run(self):
        self.zerrint_prev = 0.0

        rate = rospy.Rate(1.0 / self.Ts)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        attitude_control = AttitudeControl()
        attitude_control.run()
    except rospy.ROSInterruptException:
        pass
