#!/usr/bin/env python3
"""
    my_node.py

    A ROS node that repeats the map and odometry topic to the correct ecte477 
    namespace topics for map and path.

    Subscribed: map/, odom/
    Publishes: ecte477/map/, ecte477/path/
    Services: explore/explore_service
    Created: 2021/04/08
    Author: Brendan Halloran
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import transformations as trans
from demonstration2.msg import Beacon
from std_msgs.msg import ColorRGBA



class my_node:

    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        # sub and pub to re-route map topic
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.callback_map)
        self.map_pub = rospy.Publisher("/ecte477/map", OccupancyGrid, queue_size=1)

        # sub for frontiers, pub for goal pose
        # self.frontier_sub = rospy.Subscriber("/explore/frontiers", MarkerArray, self.callback_frontiers)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # odom sub, path pub
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.path_pub = rospy.Publisher("/ecte477/path", Path, queue_size=1)

        # keep path variable
        self.path = Path()
        self.path.header.frame_id = "odom"

        # image processing
        self.colour_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.colour_cb)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
        self.bridge = CvBridge()
        self.camera_tf = None
        # self.K = None

        # colour ranges
        self.hsv_ranges = {
            # "colour" : ((h_min, s_min, v_min), (h_max, s_max, v_max))
            "red" : ((0, 100, 0), (5, 255, 255)),
            "yellow" : ((25, 100, 0), (30, 255, 255)),
            "green" : ((50, 100, 0), (60, 255, 255)),
            "blue" : ((110, 100, 0), (130, 255, 255))
        }
        
        # beacons
        self.beacons = rospy.get_param("~beacons")
        self.marker_pub = rospy.Publisher("/ecte477/markers", MarkerArray, queue_size=100)
        self.beacon_pub = rospy.Publisher("/ecte477/beacons", Beacon, queue_size=1)
        self.published_beacons = []
        self.marker_array = MarkerArray()

    def callback_map(self, data):
        # remap map topic
        self.map_pub.publish(data)
        
    def callback_odom(self, data):
        # Turn the odometry info into a path and repeat it to the correct topic
        pose = PoseStamped()
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
        self.camera_tf = trans.msg_to_se3(data.pose.pose)

    def camera_info_cb(self, camera_info):
        # update K
        self.K = np.array(camera_info.K).reshape([3,3])

    def depth_cb(self, depth):
        # update depth image
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
   
    def colour_cb(self, image):
        # interpret data to numpy image
        arr = np.frombuffer(image.data, np.uint8)
        colour_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        
        # remove noise, convert to HSV
        blurred = cv2.GaussianBlur(colour_frame, (11,11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # track detected beacons
        detection = {"id":None, "top":None, "bottom":None, "position":None}
        top_y = 0

        # apply hsv range masking for each colour
        for colour in self.hsv_ranges.keys():
            mask = cv2.inRange(hsv, self.hsv_ranges[colour][0], self.hsv_ranges[colour][1])
            # binary erosion remove edges
            mask = cv2.erode(mask, None, iterations=2)
            # binary dilation restore size, smoothed boundaries
            mask = cv2.dilate(mask, None, iterations=2)
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            # if no contours then skip to next colour
            if len(contours) == 0:
                continue
            # take largest and fit rectangle
            largest_contour = max(contours, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(largest_contour)
            cv2.rectangle(colour_frame, (x,y), (x+w,y+h), (0,0,255), 2)
            # add to detection (direction reversed because of cv2 image coordinates)
            if y > top_y:
                # shift current top to bottom, deals with the case where bottom detected first
                detection["top"] = detection["bottom"]
                # assign new top
                detection["bottom"] = colour
                top_y = y
            else:
                # deals with the case where bottom detected second
                detection["top"] = colour

        # iterate known beacons and check if detection matches
        for b in self.beacons:
            if detection["top"] == b["top"] and detection["bottom"] == b["bottom"]:
                detection["id"] = b["id"]
                rospy.loginfo("Found %s", str(detection))

        if detection["id"] not in self.published_beacons and detection["id"] != None:
            # ensure valid beacon detected, and not already detected
            p = self.get_pixel_coord(int(x+w/2),int(y+h/2))
            detection["position"] = Point(*p)   # *p inputs each element as a separate argument [x,y,z]
            self.published_beacons.append(detection["id"])
            self.publish_beacon(detection)

    def get_pixel_coord(self, x, y):
        if type(self.K) == type(None) or type(self.camera_tf) == type(None) or type(self.depth_frame) == type(None):
            return
        depth = self.depth_frame[y][x]
        p_h = np.array([[x],[y],[1]])
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])
        p3d_w_h = np.matmul(self.camera_tf, p3d_h)
        # point 3d world coordinate
        p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])
        return p3d_w

    def publish_beacon(self, detection):
        # publish beacon
        beacon = Beacon()
        beacon.header.stamp = rospy.Time.now()
        beacon.header.frame_id = "map"
        beacon.top = detection["top"]
        beacon.bottom = detection["bottom"]
        beacon.position = detection["position"]
        self.beacon_pub.publish(beacon)
        # publish marker
        marker = Marker()
        marker.header.stamp = beacon.header.stamp
        marker.header.frame_id = beacon.header.frame_id
        marker.color = ColorRGBA(0,1,0,1)
        marker.scale = Vector3(0.25,0.25,0.25)
        marker.type = Marker.SPHERE
        marker.id = detection["id"]
        marker.pose.position = detection["position"]
        marker.pose.orientation = Quaternion(0,0,0,1)
        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)


# Main function
if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")
