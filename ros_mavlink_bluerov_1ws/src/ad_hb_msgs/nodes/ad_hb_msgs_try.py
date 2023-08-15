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
        self.relative_alt_pub = rospy.Publisher("relative_alt_msg", Float64, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.calculate_and_publish_values)

        self.trajectory_latitude = 51.5074  # Example latitude (London, UK)
        self.trajectory_longitude = -0.1278  # Example longitude (London, UK)
        self.prev_x_lat_des = 0.0
        self.prev_y_long_des = 0.0
        self.mavlink_example_override_process = None
        self.mavlink_example_heartbeat_process = None

    def calculate_and_publish_values(self, event):
        base_n_rpm = 1500
        amplitude_n_rpm = 70
        frequency_n_rpm = 0.1

        n_rpm = base_n_rpm + amplitude_n_rpm * math.sin(2 * math.pi * frequency_n_rpm * rospy.Time.now().to_sec())

        current_time = rospy.Time.now().to_sec()
        period = 40
        amplitude_z = 1.0
        frequency_z = 1.0 / period
        z_ref = 250 + amplitude_z * math.sin(2 * math.pi * frequency_z * current_time)

        amplitude_xy = 1.0
        frequency_xy = 1.0 / period
        x_lat_des_meters = 5.0 + amplitude_xy * math.sin(2 * math.pi * frequency_xy * current_time)
        x_lat_des = self.trajectory_latitude + (x_lat_des_meters / 111111.0)

        y_long_des_meters = -3.0 + amplitude_xy * math.sin(2 * math.pi * frequency_xy * current_time)
        y_long_des = self.trajectory_longitude + (y_long_des_meters / (111111.0 * math.cos(self.trajectory_latitude * math.pi / 180.0)))

        delta_x_lat_des = x_lat_des - self.prev_x_lat_des
        delta_y_long_des = y_long_des - self.prev_y_long_des
        psi_ref = math.atan2(delta_y_long_des, delta_x_lat_des)

        self.psi_ref_pub.publish(psi_ref)
        self.z_ref_pub.publish(z_ref)
        self.x_lat_des_pub.publish(x_lat_des)
        self.y_long_des_pub.publish(y_long_des)
        self.n_rpm_pub.publish(n_rpm)

        self.prev_x_lat_des = x_lat_des
        self.prev_y_long_des = y_long_des

        #if self.mavlink_example_override_process is None or self.mavlink_example_override_process.poll() is not None:
        #    mavlink_example_override_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_override.py")
        #    self.mavlink_example_override_process = subprocess.Popen(["python3", mavlink_example_override_path, str(n_rpm)])

        if self.mavlink_example_heartbeat_process is None or self.mavlink_example_heartbeat_process.poll() is not None:
            mavlink_example_heartbeat_path = os.path.expanduser("~/dev/mavros_bluerov2/ros_mavlink_bluerov_1ws/src/arm_disarm/src/mavlink_example_requesting_position.py")
            self.mavlink_example_heartbeat_process = subprocess.Popen(["python3", mavlink_example_heartbeat_path], stdout=subprocess.PIPE)
            rospy.sleep(1)  # Give the subprocess time to capture and print output
            output, _ = self.mavlink_example_heartbeat_process.communicate()

            relative_alt = None
            for line in output.decode("utf-8").splitlines():
                if "Relative Altitude:" in line:
                    relative_alt = float(line.split(":")[1].strip())
                    break

            if relative_alt is not None:
                rospy.loginfo("Extracted Relative Altitude: %.3f meters", relative_alt)
                print("Extracted Relative Altitude:", relative_alt)
                self.relative_alt_pub.publish(relative_alt)
            else:
                rospy.logwarn("Failed to extract Relative Altitude from script output.")

if __name__ == "__main__":
    node = AdHbMsgsNode()
    rospy.spin()
