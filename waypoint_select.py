import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
from scipy import spatial

def state_callback( state ):
    position = np.array([[state.pose.pose.position.x, state.pose.pose.position.y]])
    heading = state.pose.pose.position.z
    waypointSelect(np.array(), np.array(), position, heading)

state_sub = rospy.Subscriber('state', Odometry, state_callback)
target_pub = rospy.Publisher('target', Odometry, queue_size=10)

def waypointSelect(gpsDist, visionDist, currentPos, heading):
    """
    This function returns the target GPS coordinates, either a list of waypoints or an estimate from the vision system.
    The heading is also used to ensure that the waypoints behind the car are excluded
    """

    DIST_THRESHOLD = 10

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

    visionCoords = np.array(
        [[682, 2644],
         [277, 2651],
         [396, 2640]])

    currentPos = np.array([[100,100]])
    heading = np.pi/4

    gpsDist = spatial.distance.cdist(gpsCoords, currentPos)
    gpsAngle = np.arctan2(gpsCoords[:,1] - currentPos[0][1], gpsCoords[:,0] - currentPos[0][0])
    inds = np.where((gpsAngle < np.pi) & (gpsAngle > 0) & (gpsDist.flatten() > DIST_THRESHOLD))[0]
    minind = np.argmin(gpsDist[inds])
    gpsMinDist = gpsDist[minind][0]
    gpsMinAngle = gpsAngle[minind]

    visionDist = spatial.distance.cdist(visionCoords, currentPos)
    visionAngle = np.arctan2(visionCoords[:,1] - currentPos[0][1], visionCoords[:,0] - currentPos[0][0])
    inds = np.where((visionAngle < np.pi) & (visionAngle > 0) & (visionDist.flatten() > DIST_THRESHOLD))[0]
    minind = np.argmin(visionDist[inds])
    visionMinDist = visionDist[minind][0]
    visionMinAngle = visionAngle[minind]

    retind = np.argmin([gpsMinDist, visionMinDist])
    retarr = [(gpsMinDist, gpsMinAngle), (visionMinDist, visionMinAngle)]

    msg = Odometry()
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.y = gpsMinAngle

    target_pub.publish(msg)

rospy.spin()