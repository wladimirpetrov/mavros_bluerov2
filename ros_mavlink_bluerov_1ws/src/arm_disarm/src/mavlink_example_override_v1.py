#!/usr/bin/env python3

import sys
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
        *rc_channel_values)                  # RC channel list, in microseconds.

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 mavlink_example_override.py <n_rpm>")
        return

    try:
        n_rpm = float(sys.argv[1])
    except ValueError:
        print("Invalid value for n_rpm. Please provide a numeric value.")
        return



    # Now you can use the received n_rpm value in your logic
    print("Received n_rpm value:", n_rpm)
    # Set RC channel pwm values based on the n_rpm value
    
    set_rc_channel_pwm(6, n_rpm)  # For example, set yaw with some offset from n_rpm
    # ... (rest of your code)

if __name__ == "__main__":
    main()
