#!/usr/bin/env python2

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import subprocess
from math import sqrt
import time

mse_sum_distance = 0
mse_sum_theta = 0
count = 0

timer = None

realPos = (0, 0, 0)

def playBag():
    subprocess.Popen(['rosbag', 'play', '-u 100', 'bag3.bag'], cwd="/home/francesco/Desktop/bags")

def resetPose():
    subprocess.Popen(['rosservice', 'call', '/reset', '0', '0', '0'], cwd="/home/francesco/robotics")
    subprocess.Popen(['rosservice', 'call', '/reset_kinematics', '0', '0', '0'], cwd="/home/francesco/robotics")

def fromPose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)

    return (pose.position.x, pose.position.y, euler[2])

def odomCallback(msg):
    global mse_sum_distance
    global mse_sum_theta
    global count

    odomPos = fromPose(msg.pose.pose)

    #calculate distance of odom from real pose
    distance = sqrt((realPos[0] - odomPos[0])**2 + (realPos[1] - odomPos[1])**2)

    theta_diff = realPos[2] - odomPos[2]

    mse_sum_distance += distance ** 2
    mse_sum_theta += theta_diff ** 2
    count += 1


def realPoseCallback(msg):
    global realPos
    realPos = fromPose(msg.pose)

def runTest():
    global mse_sum_distance
    global mse_sum_theta
    global count

    mse_sum_distance = 0
    mse_sum_theta = 0
    count = 0

    resetPose()
    playBag()

    rospy.Timer(rospy.Duration(105), timerEnded)

def timerEnded(dc):
    try:
        line = "{},{}".format(mse_sum_distance/count, mse_sum_theta/count)
        
        rospy.loginfo(line)
    except:
        pass


def main():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber("/robot/pose", PoseStamped, realPoseCallback)

    #time.sleep(3)
    runTest()

    rospy.spin()

if __name__ == '__main__':
    main()