#!/usr/bin/env python3
import rospy
import message_filters
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path



class line_follower:
    def __init__(self, name):
        self.name = name

        # sub and pub to re-route map topic
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_cb)
        self.map_pub = rospy.Publisher("/ecte477/map", OccupancyGrid, queue_size=1)

        # odom sub, path pub
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher("/ecte477/path", Path, queue_size=1)
        self.path = Path()
        self.path.header.frame_id = "odom"

        # colour and depth topics configurable
        colour_topic = "camera/rgb/image_raw"
        depth_topic = "camera/depth/image_raw"
        # synchronised subscribers
        colour_sub = message_filters.Subscriber(colour_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10, slop=0.1)
        self.camera_sync.registerCallback(self.camera_cb)

        # image processing
        self.bridge = CvBridge()
        self.hsv_ranges = {
            # "colour" : ((h_min, s_min, v_min), (h_max, s_max, v_max))
            "red" : ((0, 100, 0), (5, 255, 255)),
            "yellow" : ((25, 100, 0), (30, 255, 255)),
            "green" : ((50, 100, 0), (60, 255, 255)),
            "blue" : ((110, 100, 0), (130, 255, 255))
        }

        # control
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # stop sign logic
        self.stopped = False
        self.stop_time = rospy.Time(0)

    def camera_cb(self, colour_msg, depth_msg):
        # collect both images in single callback
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # filter by HSV thresholding

        h, w, _ = colour_image.shape
        red_cx, _ = self.find_line(colour_image, "red")
        yellow_cx, _ = self.find_line(colour_image, "yellow")
        blue_cx, blue_cy = self.find_stop(colour_image, "blue")

        # collision avoidance
        centre_depth = depth_image[int(h/2), int(w/2)]
        if centre_depth > 0 and centre_depth < 0.5:
            self.stop()
            return

        # stop logic if blue detected
        if blue_cx != np.inf:
            depth = depth_image[blue_cy, blue_cx]
            # rospy.loginfo("blue depth: %f", depth)
            if not self.stopped and depth <= 1:
                self.stopped = True
                self.stop_time = rospy.Time.now()
                self.stop()
                return
            elif rospy.Time.now() - self.stop_time <= rospy.Duration(3):
                self.stop()
                return

        # logic to decide target based on red / yellow visibility
        print(red_cx, yellow_cx)
        if red_cx != np.inf and yellow_cx != np.inf:
            # if both valid, find the mean
            target = (red_cx + yellow_cx) / 2
        elif yellow_cx != np.inf:
            # if red not found, offset yellow
            target = yellow_cx + 700
        elif red_cx != np.inf:
            # if yellow not found, offset red
            target = red_cx - 700
        else:
            # neither found, stop
            self.stop()
            return

        # proportional control to keep white dot in centre x
        err = target - w/2
        twist = Twist()
        twist.linear.x = 0.125 # Constant forward movement
        twist.angular.z = -float(err) / 1500
        if abs(twist.angular.z) > 1000:
            # was getting an inf condition
            twist.angular.z = 0
        # publish controller output
        self.cmd_pub.publish(twist)

    def mask_colour(self, colour_image, colour):
        hsv_image = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.hsv_ranges[colour][0], self.hsv_ranges[colour][1])
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def find_centroid(self, mask):
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            return np.inf, np.inf
        return cx, cy

    def find_line(self, colour_image, colour):
        mask = self.mask_colour(colour_image, colour)
        # trim image search area
        h, w, d = colour_image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # find segment centroid
        return self.find_centroid(mask)
    
    def find_stop(self, colour_image, colour):
        mask = self.mask_colour(colour_image, colour)
        return self.find_centroid(mask)
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def map_cb(self, data):
        # remap map topic
        self.map_pub.publish(data)

    def odom_cb(self, data):
        # Turn the odometry info into a path and send it to the correct topic
        pose = PoseStamped()
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("[line_follower] Starting Line Follower Module")
    lf = line_follower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[line_follower] Shutting Down Line Follower Module")
