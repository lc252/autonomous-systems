#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class controller():
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.rate = rospy.Rate(10)

        self.move = Twist()
        self.d = 0

        self.loop()

    def scan_callback(self, scan):
        # get the distance at heading 0 degrees
        self.d = scan.ranges[0]
        rospy.loginfo(self.d)

    def proportional_velocity_controller(self, target, current, kp):
        return kp*(target - current)
        
    
    def loop(self):
        while not rospy.is_shutdown():
            vel = self.proportional_velocity_controller(1, self.d, -0.5)
            self.move.linear.x = vel
            self.pub.publish(self.move)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("tb3_wall_stop_node")
        node = controller()
    except rospy.ROSInterruptException:
        pass

