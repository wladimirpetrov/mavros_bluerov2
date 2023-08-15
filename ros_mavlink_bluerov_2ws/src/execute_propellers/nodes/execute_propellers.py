#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pymavlink import mavutil

class MavlinkController:
    def __init__(self):
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:55555')
        self.master.wait_heartbeat()

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values)

class PropellerController:
    def __init__(self):
        rospy.init_node('execute_propellers')
        self.mavlink = MavlinkController()
        self.elevator_sub = rospy.Subscriber('/bluerov2/mapped_elevator', Float64, self.elevator_callback)
        self.rudder_sub = rospy.Subscriber('/bluerov2/mapped_rudder', Float64, self.rudder_callback)

    def elevator_callback(self, msg):
        mapped_elevator = int(msg.data)
        self.mavlink.set_rc_channel_pwm(3, mapped_elevator)  # Change channel to 3 after the test

    def rudder_callback(self, msg):
        mapped_rudder = int(msg.data)
        self.mavlink.set_rc_channel_pwm(4, mapped_rudder)  # Change channel to 4 after the test

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        propeller_controller = PropellerController()
        propeller_controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        mavlink = MavlinkController()
        mavlink.set_rc_channel_pwm(1, 1500)
        mavlink.set_rc_channel_pwm(2, 1500)
        mavlink.set_rc_channel_pwm(3, 1500)
        mavlink.set_rc_channel_pwm(4, 1500)
        mavlink.set_rc_channel_pwm(5, 1500)
        mavlink.set_rc_channel_pwm(6, 1500)
        mavlink.set_rc_channel_pwm(7, 1500)
        mavlink.set_rc_channel_pwm(8, 1500)
        mavlink.set_rc_channel_pwm(9, 1100)
        mavlink.set_rc_channel_pwm(10, 1100)
        mavlink.set_rc_channel_pwm(11, 1100)
        mavlink.set_rc_channel_pwm(12, 1500)
