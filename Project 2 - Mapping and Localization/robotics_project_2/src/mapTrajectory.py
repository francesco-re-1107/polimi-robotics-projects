#!/usr/bin/env python2

from cgitb import reset
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped   
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
import os
import copy
from robotics_project_2.srv import ResetTrajectory, ResetTrajectoryResponse
from robotics_project_2.srv import SaveMapWithTrajectory, SaveMapWithTrajectoryResponse

poses = []
map = None
resolution = 50
origin = (0, 0, 0)
cwd = os.getcwd()

def poseCallback(pose):
    #add last pose to the poses list
    poses.append(pose.pose.pose.position)

def mapCallback(receivedMap):
    #convert grid map from matrix to cv2 image
    global map
    global resolution
    global origin

    origin = receivedMap.info.origin.position
    resolution = receivedMap.info.resolution
    width = receivedMap.info.width
    height = receivedMap.info.height
    size = (width , height)
    image = np.zeros(size)
    
    counter = 0
    for p in receivedMap.data:
        pixel = 100

        if p != -1:
            pixel = 255 - (p * 255/100) # p has range [0, 100]

        image[int(counter/width)][counter%width] = pixel
        counter+=1
    
    map = cv2.cvtColor(np.uint8(image),cv2.COLOR_GRAY2RGB)

def saveMapWithTrajectory(req):
    #create local copy of map
    tmpMap = copy.deepcopy(map)

    if len(poses) >= 2:
        prev_point = (int((poses[0].x - origin.x) / resolution), int((poses[0].y - origin.y) / resolution))

        for p in poses[1:]:
            end_point = (int((p.x - origin.x) / resolution), int((p.y - origin.y) / resolution))

            tmpMap = cv2.line(tmpMap, prev_point, end_point, (0, 0, 255), 1)

            prev_point = end_point

    #flip map vertically
    tmpMap = cv2.flip(tmpMap, 0)

    try:
        cv2.imwrite(req.path, tmpMap)
        rospy.loginfo("Saved map with trajectory to " + req.path)
    except:
        rospy.logerr("Could not save map to " + req.path)

    return SaveMapWithTrajectoryResponse()
    
def resetTrajectory(req):
    global poses
    poses = []

    return ResetTrajectoryResponse()

def main():
    rospy.init_node('mapTrajectory', anonymous=True)

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("map", OccupancyGrid, mapCallback)
    
    s1 = rospy.Service('reset_trajectory', ResetTrajectory, resetTrajectory)
    s2 = rospy.Service('save_map_with_trajectory', SaveMapWithTrajectory, saveMapWithTrajectory)
    
    rospy.spin()

if __name__ == '__main__':
    main()