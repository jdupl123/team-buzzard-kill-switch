import numpy as np
from scipy import spatial

def waypointSelect(gpsDist, visionDist, currentPos, heading):
    """
    This function returns the target GPS coordinates, either a list of waypoints or an estimate from the vision system.
    The heading is also used to ensure that the waypoints behind the car are excluded
    """
    DIST_THRESHOLD = 10

    gpsCoords = np.array(
        [[243, 3173],
         [525, 2997],
         [147, 130],
         [80, 80]])

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

    print retarr[retind]