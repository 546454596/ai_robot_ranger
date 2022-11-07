#!/usr/bin/env python

import os
import sys

import rospy
import math
from tf import TransformListener
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('cartopub', anonymous=True)
    pub = rospy.Publisher('/slam/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(30)
    tfl = TransformListener()
    pose_msg = PoseStamped()
    while not rospy.is_shutdown():
        if tfl.frameExists("base_link"):
            # and tfl.frameExists("map"):
            try:
                t = tfl.getLatestCommonTime("base_link", "map")
                pos, quat = tfl.lookupTransform("map", "base_link", t)
                pose_msg.header.stamp = rospy.Time.now();
                pose_msg.pose.position.x = pos[0];
                pose_msg.pose.position.y = pos[1];
                pose_msg.pose.position.z = pos[2];
                pose_msg.pose.orientation.x = quat[0];
                pose_msg.pose.orientation.y = quat[1];
                pose_msg.pose.orientation.z = quat[2];
                pose_msg.pose.orientation.w = quat[3];
                pub.publish(pose_msg)
            except:
                pass
            rate.sleep()
