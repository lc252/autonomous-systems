#!/usr/bin/env python3

import rospy
import sys
from lc252_package.srv import AddTwoInts, AddTwoIntsResponse


def add_two_ints_client(x, y):
    rospy.wait_for_service("add_two_ints")
    try:
        add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts)
        resp = add_two_ints(x,y)
        return resp.sum
    except rospy.ServiceException as e:
        print(e)

def usage():
    return "USAGE: %s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        rospy.loginfo(usage())
        sys.exit(1)

    rospy.loginfo("Requesting %s+%s"%(x,y))
    rospy.loginfo("Result: %s"%add_two_ints_client(x,y))