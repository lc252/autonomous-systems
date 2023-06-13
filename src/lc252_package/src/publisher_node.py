#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class publsiher_node():
    def __init__(self):
        self.pub = rospy.Publisher("robot_location", Point, queue_size=10)
        self.rate = rospy.Rate(10)
        # initial location
        self.location = Point(10, 10, 10)
        # start loop
        self.loop()

    def update(self):
        rospy.loginfo("Publishing location: {0}, {1}".format(self.location.x, self.location.y))    # fstrings dont work in Python2 :'(
        self.pub.publish(self.location)
        self.location.x += 1
        self.location.y += 2

    def loop(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("robot_location_publisher")
        node = publsiher_node()
    except rospy.ROSInterruptException:
        pass