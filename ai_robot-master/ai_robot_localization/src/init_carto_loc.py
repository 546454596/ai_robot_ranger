#!/usr/bin/env python

import os
import sys
import commands

import rospy
import math
from tf import TransformListener
import tf
from geometry_msgs.msg import PoseStamped

exitflag = False
# input the pose of final point of trajectory in cartographer
finalx = 0.
finaly = 0.
finalz = 0.
finalqw = 0.1
finalqx = 0.
finalqy = 0.
finalqz = 0.
finalyaw = math.atan2(2 * (finalqw * finalqz + finalqx * finalqy) , 1 - 2 * (finalqy * finalqy + finalqz * finalqz))

def cb(data):
    global exitflag, finalx, finaly, finalqw, finalqz, finalyaw
    #map frame
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w
    #final pose frame
    yaw = math.atan2(2 * (qw * qz + qx * qy) , 1 - 2 * (qy * qy + qz * qz))
    yaw = yaw - finalyaw
    fx = (x - finalx)*math.cos(finalyaw) + (y - finaly)*math.sin(finalyaw)
    fy = (x - finalx)*math.sin(-finalyaw) + (y - finaly)*math.cos(finalyaw)
    cmdstr = 'rosrun cartographer_ros cartographer_start_trajectory -configuration_directory \'/home/mediasoc/catkin_ws/src/ai_robot/ai_robot_localization/carto_configs\' -configuration_basename \'carto_hw_loc.lua\' -initial_pose \'{{to_trajectory_id = 0, relative_pose = {{ translation = {{ {}, {}, {} }}, rotation = {{ {}, {}, {} }} }} }}\''.format(fx,fy,0, 0,0,yaw)
    commands.getstatusoutput(cmdstr)
    # print(finalx, finaly, finalqw, finalqz, finalyaw)
    exitflag = True

if __name__ == '__main__':
    if(len(sys.argv) > 1):
        last_pose_file = open(sys.argv[1], 'r')
        strs = last_pose_file.readline()
        strs = strs.split(',')
        finalx = float(strs[0])
        finaly = float(strs[1])
        finalqw = float(strs[3])
        finalqz = float(strs[6])
        finalyaw = math.atan2(2 * (finalqw * finalqz + finalqx * finalqy) , 1 - 2 * (finalqy * finalqy + finalqz * finalqz))
    rospy.init_node('init_cartoloc', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, cb)
    rospy.loginfo('[ATTENTION]be sure that the brain_node is not on and please use 2D Nav Goal buttom to set init pose for localization...')
    while not rospy.is_shutdown() and not exitflag:
        rate.sleep()
    rospy.loginfo('set init pose done!')
