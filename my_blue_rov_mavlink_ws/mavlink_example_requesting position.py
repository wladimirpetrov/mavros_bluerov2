import time
import sys

from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait for a heartbeat before sending commands
master.wait_heartbeat()

# Request the global position
master.mav.global_position_int_send(
    int(time.time() * 1e3),  # time_boot_ms (milliseconds since boot)

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

while True:
    time.sleep(0.1)
    try:
        message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        lat = message['lat'] * 1e-7  # Latitude in degrees
        lon = message['lon'] * 1e-7  # Longitude in degrees
        alt = message['alt'] * 1e-3  # Altitude in meters
        print('Latitude: %.7f, Longitude: %.7f, Altitude: %.3f meters' % (lat, lon, alt))
    except Exception as error:
        print(error)
        sys.exit(0)
