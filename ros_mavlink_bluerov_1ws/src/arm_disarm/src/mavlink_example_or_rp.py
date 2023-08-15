#!/usr/bin/env python3

import sys
import select
import threading
from pymavlink import mavutil

# Connect to the MAVLink
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait for heartbeat before sending commands
master.wait_heartbeat()

def set_rc_channel_pwm(channel_id, pwm=1500):
    
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return
    
    # Convert pwm to integer
    pwm = int(pwm)

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)    

def feedback_thread():
    while True:
        try:
            inputs = [master.port]
            readable, _, _ = select.select(inputs, [], [], 0.1)

            if master.port in readable:
                message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
                lat = message['lat'] * 1e-7
                lon = message['lon'] * 1e-7
                alt = message['alt'] * 1e-3
                relative_alt = message['relative_alt'] * 1e-3
                print('Latitude: %.7f, Longitude: %.7f, Altitude: %.3f meters, Relative Altitude: %.3f meters' % (lat, lon, alt, relative_alt))
        except Exception as error:
            print(error)
            sys.exit(0)

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 mavlink_example_override.py <n_rpm>")
        return

    try:
        n_rpm = float(sys.argv[1])
    except ValueError:
        print("Invalid value for n_rpm. Please provide a numeric value.")
        return

    print("Received n_rpm value:", n_rpm)
    set_rc_channel_pwm(5, n_rpm)

if __name__ == "__main__":
    feedback_thread = threading.Thread(target=feedback_thread)
    feedback_thread.daemon = True  # Exit the thread when the main program exits
    feedback_thread.start()

    main()
