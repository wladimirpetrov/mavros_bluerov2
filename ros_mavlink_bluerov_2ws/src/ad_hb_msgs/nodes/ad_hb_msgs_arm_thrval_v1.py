#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import subprocess
import os

class AdHbMsgsNode:
    def __init__(self):
        rospy.init_node("ad_hb_msgs")
        self.psi_ref_pub = rospy.Publisher("psi_ref_msg", Float64, queue_size=10)
        self.z_ref_pub = rospy.Publisher("z_ref_msg", Float64, queue_size=10)
        self.x_lat_des_pub = rospy.Publisher("x_des_msg", Float64, queue_size=10)
        self.y_long_des_pub = rospy.Publisher("y_des_msg", Float64, queue_size=10)
        self.n_rpm_pub = rospy.Publisher("n_rpm_msg", Float64, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.calculate_and_publish_values)

        # Geographic location of the trajectory (latitude, longitude)
        self.trajectory_latitude = 51.5074  # Example latitude (London, UK)
        self.trajectory_longitude = -0.1278  # Example longitude (London, UK)

        # Previous x_lat_des and y_long_des values for calculating heading angle
        self.prev_x_lat_des = 0.0
        self.prev_y_long_des = 0.0

        # Flag to indicate if the other script has been launched
        self.mavlink_example_override_launched = False

    def calculate_and_publish_values(self, event):
        
        # Calculate n_rpm as a sine function with a base value of 1400 and oscillation of +-50
        base_n_rpm = 1550
        amplitude_n_rpm = 20
        frequency_n_rpm = 1  # Adjust this value as needed

        # Calculate n_rpm using the sine function
        n_rpm = base_n_rpm + amplitude_n_rpm * math.sin(2 * math.pi * frequency_n_rpm * rospy.Time.now().to_sec())


        # Calculate the current time in seconds
        current_time = rospy.Time.now().to_sec()

        # Calculate the desired depth using a sine function
        period = 40
        amplitude_z = 1.0
        frequency_z = 1.0 / period
        z_ref = 250 + amplitude_z * math.sin(2 * math.pi * frequency_z * current_time)

        # Calculate the desired x and y coordinates using different sine functions
        amplitude_xy = 1.0  # Adjust this value as needed
        frequency_xy = 1.0 / period  # Adjust this value as needed

        # Replace x_des with x_lat_des and convert meters to degrees latitude
        x_lat_des_meters = 5.0 + amplitude_xy * math.sin(2 * math.pi * frequency_xy * current_time)
        x_lat_des = self.trajectory_latitude + (x_lat_des_meters / 111111.0)

        # Replace y_des with y_long_des and convert meters to degrees longitude
        y_long_des_meters = -3.0 + amplitude_xy * math.sin(2 * math.pi * frequency_xy * current_time)
        y_long_des = self.trajectory_longitude + (y_long_des_meters / (111111.0 * math.cos(self.trajectory_latitude * math.pi / 180.0)))

        # Calculate the change in x_lat_des and y_long_des
        delta_x_lat_des = x_lat_des - self.prev_x_lat_des
        delta_y_long_des = y_long_des - self.prev_y_long_des

        # Calculate the desired heading angle (psi_ref) based on the changes in x_lat_des and y_long_des
        psi_ref = math.atan2(delta_y_long_des, delta_x_lat_des)

        # Publish psi_ref, z_ref, x_lat_des, y_long_des, and n_rpm values to their respective topics
        self.psi_ref_pub.publish(psi_ref)
        self.z_ref_pub.publish(z_ref)
        self.x_lat_des_pub.publish(x_lat_des)
        self.y_long_des_pub.publish(y_long_des)
        self.n_rpm_pub.publish(n_rpm)

        # Update the previous values for the next iteration
        self.prev_x_lat_des = x_lat_des
        self.prev_y_long_des = y_long_des

        # Launch mavlink_example_override.py with n_rpm as a command-line argument
        if not self.mavlink_example_override_launched:
            mavlink_example_override_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_override.py")
            subprocess.Popen(["python3", mavlink_example_override_path, str(n_rpm)])
            self.mavlink_example_override_launched = True

        rospy.loginfo("We are here:")

if __name__ == "__main__":
    node = AdHbMsgsNode()
    rospy.spin()
