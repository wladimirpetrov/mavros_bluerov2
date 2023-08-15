# Import mavutil
from pymavlink import mavutil
import rospy
from std_msgs.msg import Float64

# Callback function for handling the received psi_ref messages
def psi_ref_callback(msg):
    # Handle the received psi_ref message here
    print("Received psi_ref:", msg.data)

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Subscribe to the psi_ref_msg topic
rospy.init_node("mavlink_subscriber")
rospy.Subscriber("psi_ref_msg", Float64, psi_ref_callback)

# Wait for some time to allow the subscriber to initialize
rospy.sleep(1)

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Now start the rospy loop (This will keep the program running until it's terminated)
rospy.spin()
