import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
from scipy import spatial

def state_callback( state ):
    position = np.array([[state.pose.pose.position.x, state.pose.pose.position.y]])
    heading = state.pose.pose.position.z
    waypointSelect(position, heading)

state_sub = rospy.Subscriber('state', Odometry, state_callback)
target_pub = rospy.Publisher('target', Odometry, queue_size=10)

def waypointSelect(currentPos, heading):
    """
    This function returns the target GPS coordinates, either a list of waypoints or an estimate from the vision system.
    The heading is also used to ensure that the waypoints behind the car are excluded
    """
    gpsCoords = np.array([
        [-27.8552175, 153.1511374],
        [-27.8554650, 153.1516188],
        [-27.8557407, 153.1520735],
        [-27.8560205, 153.1521694],
        [-27.8560920, 153.1519464],
        [-27.8559772, 153.1513396],
        [-27.8555771, 153.1508628],
        [-27.8553002, 153.1509104]
    ])
    DIST_THRESHOLD = 10

    gpsDist = spatial.distance.cdist(gpsCoords, currentPos)
    gpsAngle = np.arctan2(gpsCoords[:,1] - currentPos[0][1], gpsCoords[:,0] - currentPos[0][0])
    inds = np.where((gpsAngle < np.pi) & (gpsAngle > 0) & (gpsDist.flatten() > DIST_THRESHOLD))[0]
    minind = np.argmin(gpsDist[inds])
    gpsMinX = gpsCoords[minind][0]
    gpsMinY = gpsCoords[minind][1]
    gpsMinAngle = gpsAngle[minind]

    print (gpsMinX, gpsMinY, gpsMinAngle)

    msg = Odometry()
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.z = gpsMinAngle
    msg.twist.twist.linear.z = 10

    target_pub.publish(msg)
rospy.init_node('waypoint_selector', anonymous=False)
rospy.spin()

# Dummy data for testing











# visionCoords = np.array(
#     [[682, 2644],
#      [277, 2651],
#      [396, 2640]])

# currentPos = np.array([[100,100]])
# heading = np.pi/4