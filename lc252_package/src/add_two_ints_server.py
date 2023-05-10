#!/usr/bin/env python3

import rospy
from lc252_package.srv import AddTwoInts, AddTwoIntsResponse


def handle(req):
    rospy.loginfo("Returning [%s + %s = %s]"%(req.a, req.b, req.a+req.b))
    return AddTwoIntsResponse(req.a+req.b)

def add_two_ints_server():
    rospy.init_node("add_two_ints_server")
    s = rospy.Service("add_two_ints", AddTwoInts, handle)
    rospy.loginfo("Ready")
    rospy.spin()


if __name__ == '__main__':
    add_two_ints_server()