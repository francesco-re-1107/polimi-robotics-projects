#!/usr/bin/env python2

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import subprocess
from math import sqrt
import time
import message_filters

param_start = 0.0018120
param_end =   0.0018140
param_step =  0.0000001
current_param = param_start

mse_sum_distance = 0
mse_sum_theta = 0
count = 0

g_count = 0

timer = None

realPos = (0, 0, 0)

def playBag():
    subprocess.Popen(['rosbag', 'play', '-u 100', 'bag3.bag'], cwd="/home/francesco/Desktop/bags")
    subprocess.Popen(['rosservice', 'call', '/reset_kinematics', '0', '0', '0'], cwd="/home/francesco/robotics")

def resetPose():
    subprocess.Popen(['rosservice', 'call', '/reset', '0', '0', '0'], cwd="/home/francesco/robotics")

def setParams():
    #rosrun dynamic_reconfigure dynparam set kinematics cpr
    #subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', 'kinematics', 'cpr', str(current_cpr)], cwd="/home/francesco/robotics")
    subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', 'kinematics', 'wheels_radius', str(current_param * 42.0)], cwd="/home/francesco/robotics")
    time.sleep(2)

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


def callback(odom_msg, gt_pose_msg):
    global mse_sum_distance
    global mse_sum_theta
    global count
    
    gtPos = fromPose(gt_pose_msg.pose)
    odomPos = fromPose(odom_msg.pose.pose)

    rospy.loginfo("GT: %s ODOM: %s", gtPos, odomPos)

    #calculate distance of odom from real pose
    distance = sqrt((gtPos[0] - odomPos[0])**2 + (gtPos[1] - odomPos[1])**2)
    theta_diff = gtPos[2] - odomPos[2]

    mse_sum_distance += distance ** 2
    mse_sum_theta += theta_diff ** 2
    count += 1
    

def nextIteration(first=False):
    global mse_sum_distance
    global mse_sum_theta
    global count
    global current_param
    global g_count

    mse_sum_distance = 0
    mse_sum_theta = 0
    count = 0

    if current_param >= param_end:
        rospy.loginfo("FINISHED")
        return

    if not first:
        current_param += param_step
        g_count += 1

    setParams()

    resetPose()
    
    playBag()

    rospy.Timer(rospy.Duration(105), timerEnded)

def timerEnded(dc):
    try:
        line = "{},{},{},{}".format(g_count, current_param, mse_sum_distance/count, mse_sum_theta/count)
        
        #save data
        with open("/home/francesco/calibration.csv", "a") as f:
            f.write(line + "\n")

        rospy.loginfo(line)

        nextIteration()
    except:
        pass


def main():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.Subscriber("/robot/pose", PoseStamped, realPoseCallback)

    """odom_sub = message_filters.Subscriber('/odom', Odometry)
    gt_pose_sub = message_filters.Subscriber('/robot/pos', PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, gt_pose_sub], 1000, 2.5)
    ts.registerCallback(callback)"""

    nextIteration(first=True)

    rospy.spin()

if __name__ == '__main__':
    main()