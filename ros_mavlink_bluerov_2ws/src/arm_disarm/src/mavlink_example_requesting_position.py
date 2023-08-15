import sys
import select
import time
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:55555')
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

while True:
    try:
        # Wait for incoming data for up to 0.1 seconds
        inputs = [master.port]
        readable, _, _ = select.select(inputs, [], [], 0.1)

        if master.port in readable:
            message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
            lat = message['lat'] * 1e-7  # Latitude in degrees
            lon = message['lon'] * 1e-7  # Longitude in degrees
            alt = message['alt'] * 1e-3  # Altitude in meters
            relative_alt = message['relative_alt'] * 1e-3  # Relative altitude in meters
            print('Latitude: %.7f, Longitude: %.7f, Altitude: %.3f meters, Relative Altitude: %.3f meters' % (lat, lon, alt, relative_alt))
        # time.sleep(0.1)
    except Exception as error:
        print(error)
        sys.exit(0)
