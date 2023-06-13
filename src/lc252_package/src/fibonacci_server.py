#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from lc252_package.msg import FibonacciAction, FibonacciFeedback, FibonacciResult


class fibonacci_server:
    def __init__(self, name):
        self.name = name
        self.feedback = FibonacciFeedback()
        self.result = FibonacciResult()
        self.action_server = SimpleActionServer(self.name, FibonacciAction, self.callback, auto_start=False)
        self.action_server.start()

    def callback(self, goal):
        rate = rospy.Rate(1)
        success = True
        self.feedback.sequence[:]= []       # clear feedback
        self.feedback.sequence.append(0)    # init with 0,1
        self.feedback.sequence.append(1)
        rospy.loginfo("[{}] Excecuting, creating fibonacci sequence of order {} with seeds {},{}".format(
            self.name, goal.order, self.feedback.sequence[0], self.feedback.sequence[1]
        ))

        for i in range(1, goal.order):
            # check if goal pre-empted or ROS down
            if self.action_server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo("Pre-empted")
                success = False
                self.action_server.set_preempted()
                break

            self.feedback.sequence.append(self.feedback.sequence[i-1]+self.feedback.sequence[i])
            self.action_server.publish_feedback(self.feedback)
            rate.sleep()

        if success:
            self.result.sequence = self.feedback.sequence
            rospy.loginfo("Success")
            self.action_server.set_succeeded(self.result)



if __name__ == "__main__":
    print "Starting Server"
    rospy.init_node("fibonacci_server", anonymous=True, log_level=rospy.DEBUG)
    fib = fibonacci_server("fibber")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Quit"

    