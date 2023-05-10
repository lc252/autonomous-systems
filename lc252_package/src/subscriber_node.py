#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point


class subscriber_node():
    def __init__(self):
        rospy.Subscriber("robot_location", Point, self.callback)
        rospy.spin()

    def callback(self, location):
        # location is of type geometry_msgs/Point
        rospy.loginfo("Received location: {0}, {1}".format(location.x, location.y))    # fstrings dont work in Python2 :'(


if __name__ == "__main__":
    try:
        rospy.init_node("robot_location_listener")
        node = subscriber_node()
    except:
        pass