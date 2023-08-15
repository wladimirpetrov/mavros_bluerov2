#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

class AttitudeControl:
    def __init__(self):
        rospy.init_node('attitude_control')

        self.z_ref_sub = rospy.Subscriber('/z_ref_msg', Float64, self.z_ref_callback)
        self.psi_ref_sub = rospy.Subscriber('/psi_ref_msg', Float64, self.psi_ref_callback)
        self.x_des_sub = rospy.Subscriber('/x_des_msg', Float64, self.x_des_callback)
        self.y_des_sub = rospy.Subscriber('/y_des_msg', Float64, self.y_des_callback)
        self.lat_sub = rospy.Subscriber('/bluerov2/latitude', Float64, self.lat_callback)
        self.lon_sub = rospy.Subscriber('/bluerov2/longitude', Float64, self.lon_callback)
        self.alt_sub = rospy.Subscriber('/bluerov2/altitude', Float64, self.alt_callback)
        self.rel_alt_sub = rospy.Subscriber('/bluerov2/relative_altitude', Float64, self.rel_alt_callback)
        self.roll_sub = rospy.Subscriber('/bluerov2/roll', Float64, self.roll_callback)
        self.pitch_sub = rospy.Subscriber('/bluerov2/pitch', Float64, self.pitch_callback)
        self.yaw_sub = rospy.Subscriber('/bluerov2/yaw', Float64, self.yaw_callback)

        # Create a publisher for the mapped elevator value
        self.mapped_elevator_pub = rospy.Publisher('/bluerov2/mapped_elevator', Float64, queue_size=10)
        self.mapped_rudder_pub = rospy.Publisher('/bluerov2/mapped_rudder', Float64, queue_size=10)

        self.Ts = 0.05
        self.z_ref = 0.0
        self.psi_ref = 0.0
        self.x_des = 0.0
        self.y_des = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.rel_alt = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        

        self.zerr_prev = 0.0
        self.zerrint_prev = 0.0
        self.psierr_prev = 0.0
        self.psierrint_prev = 0.0


        self.kp_elevator = rospy.get_param("~kp_elevator", 9.0)
        self.ki_elevator = rospy.get_param("~ki_elevator", 0.0)
        self.kd_elevator = rospy.get_param("~kd_elevator", 5.0)

        self.kp_rudder = rospy.get_param("~kp_rudder", 1.0)
        self.ki_rudder = rospy.get_param("~ki_rudder", 0.0)
        self.kd_rudder = rospy.get_param("~kd_rudder", 1.0)

    def z_ref_callback(self, msg):
        self.z_ref = msg.data
        #print("Received z_ref:", self.z_ref)

    def psi_ref_callback(self, msg):
        self.psi_ref = msg.data

    def x_des_callback(self, msg):
        self.x_des = msg.data

    def y_des_callback(self, msg):
        self.y_des = msg.data
    
    def lat_callback(self, msg):
        self.lat = msg.data

    def lon_callback(self, msg):
        self.lon = msg.data

    def alt_callback(self, msg):
        self.alt = msg.data

    def rel_alt_callback(self, msg):
        self.rel_alt = msg.data      

    def roll_callback(self, msg):
        self.roll = msg.data

    def pitch_callback(self, msg):
        self.pitch = msg.data   

    def yaw_callback(self, msg):

        self.yaw = msg.data
        print("Actual yaw in radians:", self.yaw)
        #print("Received Relative Altitude:", self.rel_alt)

        self.roll = math.degrees(self.roll)
        self.pitch = math.degrees(self.pitch)
        self.yaw = math.degrees(msg.data)
        
        print("Actual yaw in degrees:", self.yaw)

        self.z = self.rel_alt
        self.x = self.alt
        self.y = self.lon
        self.psi = self.yaw

        x_error = self.x_des - self.lon
        y_error = self.y_des - self.alt
        z_error = self.z_ref - self.rel_alt
        #print("z_error:", z_error)

        zerr = z_error
        zerrint = self.zerrint_prev + z_error * self.Ts
        elevator = self.limit_elevator(-self.PID(zerr, self.kp_elevator, self.ki_elevator, self.kd_elevator, self.zerr_prev, zerrint))

        psierr = self.psi_ref - self.psi
        psierrint = self.psierrint_prev + psierr * self.Ts        
        rudder = self.limit_rudder(self.PID(psierr, self.kp_rudder, self.ki_rudder, self.kd_rudder, self.psierr_prev, psierrint))

        mapped_elevator = int(1500 + (elevator * 30))  # Example mapping, adjust as needed
        self.mapped_elevator_pub.publish(mapped_elevator)
        #print("Received Relative Altitude:", mapped_elevator)

        mapped_rudder = int(1500 + (rudder * 5))  # Example mapping, adjust as needed
        self.mapped_rudder_pub.publish(mapped_rudder)        
        print("Mapped_rudder:", mapped_rudder)

        self.zerr_prev = z_error
        self.zerrint_prev = zerrint
        self.psierr_prev = psierr
        self.psierrint_prev = psierrint        

    def PID(self, error, kp, ki, kd, prev_err, integral):
        integral = integral + error * self.Ts
        integral = max(min(integral, 5.0), -5.0)
        derivative = (error - prev_err) / self.Ts
        output = kp * error + ki * integral + kd * derivative
        return output

    def limit_elevator(self, value):
        if value >= 20.0:
            return 20.0
        elif value <= -20.0:
            return -20.0
        else:
            return value
        
    def limit_rudder(self, value):
        if value >= 20.0:
            return 20.0
        elif value <= -20.0:
            return -20.0
        else:
            return value        

    def run(self):
        rate = rospy.Rate(1.0 / self.Ts)
        while not rospy.is_shutdown():
            # Add your control loop logic here...
            rate.sleep()

if __name__ == '__main__':
    attitude_control = AttitudeControl()
    attitude_control.run()
