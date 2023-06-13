#!/usr/bin/env python3
"""
    my_node.py

    A ROS node that repeats the map and odometry topic to the correct ecte477 
    namespace topics for map and path.

    Subscribed: map/, odom/
    Publishes: ecte477/map/, ecte477/e_path/, ecte477/r_path/
    Services: explore/explore_service
    Created: 2021/04/08
    Author: Brendan Halloran
    Updated 12/04/2022 by Jeff Moscrop
"""

import rospy
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import SetBool, SetBoolRequest



class my_node:
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        
        # sub and pub to re-route map topic
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.callback_map)
        self.map_pub = rospy.Publisher("/ecte477/maze", OccupancyGrid, queue_size=1)

        # sub for frontiers, pub for goal pose
        self.frontier_sub = rospy.Subscriber("/explore/frontiers", MarkerArray, self.callback_frontiers)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # odom sub, path pubs
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.e_path_pub = rospy.Publisher("/ecte477/e_path", Path, queue_size=1)
        self.r_path_pub = rospy.Publisher("/ecte477/r_path", Path, queue_size=1)

        # track whether exploring or returning
        self.exploring = True

        # keep path variables
        self.e_path = Path()
        self.e_path.header.frame_id = "odom"
        self.r_path = Path()
        self.r_path.header.frame_id = "odom"
		
        # wait 8 sec
        t0 = rospy.Time.now()
        while rospy.Time().now() - t0 < rospy.Duration(8):
             pass
        # call the service to start exploring
        try:
            rospy.wait_for_service("explore/explore_service")
            start_explore_srv = rospy.ServiceProxy("explore/explore_service", SetBool)
            explore_srv_obj = SetBoolRequest()
            explore_srv_obj.data = True
            resp = start_explore_srv(explore_srv_obj)
            rospy.loginfo("Response: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Error: %s", e)


    def callback_map(self, data):
        # re-publish the map on new topic
        self.map_pub.publish(data)
        
    def callback_odom(self, data):
        # listen to odometry and add the current pose to either the exploration or returning path
        current_pose = PoseStamped()
        current_pose.header.stamp = rospy.Time.now()
        current_pose.header.frame_id = "odom"
        current_pose.pose = data.pose.pose
    
        # add pose to path and publish
        if self.exploring:
            self.e_path.poses.append(current_pose)
            self.e_path_pub.publish(self.e_path)
        else:
            # returning
            self.r_path.poses.append(current_pose)
            self.r_path_pub.publish(self.r_path)
        
    def callback_frontiers(self, frontiers):
        # Detect when exploration is finished, then return home
        # exploration is finished when there are no frontiers, i.e. the length of markers array is 0
        if len(frontiers.markers) == 0 and self.exploring:
            rospy.loginfo("Exploration complete, returning home.")
            # wait 5 seconds
            t0 = rospy.Time.now()
            while rospy.Time().now() - t0 < rospy.Duration(5):
                # do nothing
                pass

            # set a goal back at home position
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position = Point(0, 0, 0)
            goal.pose.orientation = Quaternion(0, 0, 0, 1)

            # publish goal
            self.goal_pub.publish(goal)

            # now returning
            self.exploring = False
	    

# Main function
if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")
