import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandBoolResponse

# ROS node initialization
rospy.init_node('arm_disarm_node')

# Service client for arming/disarming
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

# Function to arm the vehicle
def arm_vehicle():
    rospy.wait_for_service('/mavros/cmd/arming')
    
    # Create a request to arm the vehicle
    req = CommandBoolRequest()
    req.value = True

    try:
        # Call the arming service
        response = arming_client.call(req)
        if response.success:
            rospy.loginfo("Vehicle armed!")
        else:
            rospy.logwarn("Arming failed!")
    except rospy.ServiceException as e:
        rospy.logwarn("Arming service call failed: %s", str(e))

# Function to disarm the vehicle
def disarm_vehicle():
    rospy.wait_for_service('/mavros/cmd/arming')

    # Create a request to disarm the vehicle
    req = CommandBoolRequest()
    req.value = False

    try:
        # Call the disarming service
        response = arming_client.call(req)
        if response.success:
            rospy.loginfo("Vehicle disarmed!")
        else:
            rospy.logwarn("Disarming failed!")
    except rospy.ServiceException as e:
        rospy.logwarn("Disarming service call failed: %s", str(e))

# Arm the vehicle
arm_vehicle()

# Perform actions or tasks while the vehicle is armed

# Disarm the vehicle
disarm_vehicle()
