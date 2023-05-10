#!/usr/bin/env python

from logging import log
import rospy
from actionlib import SimpleActionClient
from lc252_package.msg import FibonacciAction, FibonacciGoal


class fibonacci_client():
    def __init__(self):
        self.action_client = SimpleActionClient("fibber", FibonacciAction)
        self.action_client.wait_for_server()
        
        goal = FibonacciGoal(order=20)
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

        result = self.action_client.get_result()
        rospy.loginfo("Result: {}".format(", ".join([str(n) for n in result.sequence])))


if __name__ == "__main__":
    print "Starting Client"
    rospy.init_node("fibonacci_client", anonymous=True, log_level=rospy.DEBUG)
    fib = fibonacci_client()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Down"