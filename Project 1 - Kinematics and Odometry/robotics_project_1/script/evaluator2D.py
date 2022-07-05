#!/usr/bin/env python2

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import subprocess
from math import sqrt
import time

cpr_start = 41.00
cpr_end = 43.00
cpr_step = 0.125
current_cpr = cpr_start

wr_start = 0.060
wr_end = 0.080
wr_step = 0.002
current_wr = wr_start

mse_sum_distance = 0
mse_sum_theta = 0
count = 0

g_count = 0

timer = None

realPos = (0, 0, 0)

def playBag():
    subprocess.Popen(['rosbag', 'play', '-u 100', 'bag3.bag'], cwd="/home/francesco/Desktop/bags")

def resetPose():
    subprocess.Popen(['rosservice', 'call', '/reset', '0', '0', '0'], cwd="/home/francesco/robotics")
    subprocess.Popen(['rosservice', 'call', '/reset_kinematics', '0', '0', '0'], cwd="/home/francesco/robotics")

def setParams():
    #rosrun dynamic_reconfigure dynparam set kinematics cpr
    subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', 'kinematics', 'cpr', str(current_cpr)], cwd="/home/francesco/robotics")
    subprocess.Popen(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', 'kinematics', 'wheels_radius', str(current_wr)], cwd="/home/francesco/robotics")
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

def nextIteration(first=False):
    global mse_sum_distance
    global mse_sum_theta
    global count
    global current_cpr
    global current_wr
    global g_count

    mse_sum_distance = 0
    mse_sum_theta = 0
    count = 0

    if current_cpr >= cpr_end and current_wr >= wr_end:
        rospy.loginfo("FINISHED")
        return

    if not first:
        if current_cpr < cpr_end:
            if current_wr < wr_end:
                current_wr += wr_step
            else:
                current_cpr += cpr_step
                current_wr = wr_start

        g_count += 1

    setParams()

    resetPose()
    
    playBag()

    rospy.Timer(rospy.Duration(105), timerEnded)

def timerEnded(dc):
    try:
        line = "{},{},{},{},{}".format(g_count,current_cpr, current_wr, mse_sum_distance/count, mse_sum_theta/count)
        
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

    nextIteration(first=True)

    rospy.spin()

if __name__ == '__main__':
    main()