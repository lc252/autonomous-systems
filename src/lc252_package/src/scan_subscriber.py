#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(len(msg.ranges))

rospy.init_node("read_lidar")

sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin()