#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("move_tutlebot3_node")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.5

for i in range(0, 20):
    pub.publish(move)
    rate.sleep()

while not rospy.is_shutdown():
    num_conns = pub.get_num_connections()
    if num_conns > 0:
        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)
        rospy.loginfo("cmd_vel published")
        break
    else:
        rate.sleep()
