#!/usr/bin/env python

import time
import sys
import rospy
from std_msgs.msg import Float64

from pymavlink import mavutil

# Initialize ROS node
rospy.init_node('bluerov2_position_publisher')

# Create ROS publishers for latitude, longitude, altitude, and relative altitude
lat_publisher = rospy.Publisher('/bluerov2/latitude', Float64, queue_size=10)
lon_publisher = rospy.Publisher('/bluerov2/longitude', Float64, queue_size=10)
alt_publisher = rospy.Publisher('/bluerov2/altitude', Float64, queue_size=10)
rel_alt_publisher = rospy.Publisher('/bluerov2/relative_altitude', Float64, queue_size=10)

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14555')
# Wait for a heartbeat before sending commands
master.wait_heartbeat()

# Request the global position
master.mav.global_position_int_send(
    0,   # time_boot_ms (not used)
    0,   # lat (not used)
    0,   # lon (not used)
    0,   # alt (not used)
    0,   # relative_alt (not used)
    0,   # vx (not used)
    0,   # vy (not used)
    0,   # vz (not used)
    0,   # hdg (not used)
    0    # coordinate_frame (not used)
)

while not rospy.is_shutdown():
    time.sleep(0.1)  # Delay before the next iteration
    try:
        message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        lat = message['lat'] * 1e-7  # Latitude in degrees
        lon = message['lon'] * 1e-7  # Longitude in degrees
        alt = message['alt'] * 1e-3  # Altitude in meters
        relative_alt = message['relative_alt'] * 1e-3  # Relative altitude in meters
        
        # Publish values through ROS
        lat_publisher.publish(lat)
        lon_publisher.publish(lon)
        alt_publisher.publish(alt)
        rel_alt_publisher.publish(relative_alt)
        

    except Exception as error:
        print(error)
        sys.exit(0)
