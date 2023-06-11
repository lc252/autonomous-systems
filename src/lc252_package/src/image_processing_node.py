#!/usr/bin/env python3

import rospy
import cv2
import imutils
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import transformations as trans



class image_processing_node():
    def __init__(self, name):
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.mask = None
        self.largest_contour = None
        self.num_colour_images = 0
        self.num_depth_images = 0

        self.beacons = rospy.get_param("~beacons")

        self.colour_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.colour_cb)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        # colour ranges
        self.hsv_ranges = {
            # "colour" : ((h_min, s_min, v_min), (h_max, s_max, v_max))
            "red" : ((0, 100, 0), (5, 255, 255)),
            "yellow" : ((25, 100, 0), (30, 255, 255)),
            "green" : ((50, 100, 0), (60, 255, 255)),
            "blue" : ((110, 100, 0), (130, 255, 255))
        }

        self.camera_transform = None
        self.K = None

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.render()
            r.sleep()


    def colour_cb(self, image):
        # interpret data to numpy image
        arr = np.frombuffer(image.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        # remove noise, convert to HSV
        blurred = cv2.GaussianBlur(frame, (11,11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # track detected beacons
        detection = {"id":None, "top":None, "bottom":None}
        top_y = 0

        # apply hsv range masking for each colour
        for colour in self.hsv_ranges.keys():
            self.mask = cv2.inRange(hsv, self.hsv_ranges[colour][0], self.hsv_ranges[colour][1])
            # binary erosion remove edges
            self.mask = cv2.erode(self.mask, None, iterations=2)
            # binary dilation restore size, smoothed boundaries
            self.mask = cv2.dilate(self.mask, None, iterations=2)
            contours = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            # if no contours then skip to next colour
            if len(contours) == 0:
                continue
            # take largest and fit rectangle
            self.largest_contour = max(contours, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(self.largest_contour)
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
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

        for b in self.beacons:
            if detection["top"] == b["top"] and detection["bottom"] == b["bottom"]:
                detection["id"] = b["id"]

        if detection["id"] != None:
            point = self.get_pixel_depth(int(x+w/2),int(y+h/2))

        self.colour_frame = frame


    def depth_cb(self, depth):
        # rospy.loginfo("Depth")
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    def camera_info_cb(self, camera_info):
        self.K = np.array(camera_info.K).reshape([3,3])

    def odom_cb(self, odometry):
        self.camera_transform = trans.msg_to_se3(odometry.pose.pose)

    def get_pixel_depth(self, x, y):
        if type(self.K) == type(None) or type(self.camera_transform) == type(None) or type(self.depth_frame) == type(None):
            return
        depth = self.depth_frame[y][x]
        p_h = np.array([[x],[y],[1]])
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])
        p3d_w_h = np.matmul(self.camera_transform, p3d_h)
        # point 3d world coordinate
        p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])
        print(p3d_w)


    def render(self):
        # checking if an array is None is not recommended in python2 and deprecated in python3 because it is ambiguous. Instead use type(), more robust.
        if type(self.colour_frame) != type(None) or type(self.depth_frame) != type(None) or type(self.mask) != type(None):
            # cv2.imshow("Colour", self.colour_frame)
            # cv2.imshow("Depth", self.depth_frame)
            # cv2.imshow("filtered", self.mask)

            # colour_cv_frame = cv2.rectangle(colour_cv_frame, (x,y), (x+w, y+h), (0, 0, 255), 2)

            resp = cv2.waitKey(80)
            if resp == ord("c"):
                rospy.loginfo("Saving Colour Image")
                cv2.imwrite("/home/secte/catkin_ws/src/lc252_package/images/colour_{}.png".format(self.num_colour_images), self.colour_frame)
                self.num_colour_images += 1
            if resp == ord("d"):
                rospy.loginfo("Saving Depth Image")
                cv2.imwrite("/home/secte/catkin_ws/src/lc252_package/images/depth_{}.png".format(self.num_depth_images), self.depth_frame)
                self.num_depth_images += 1


if __name__ == "__main__":
    rospy.init_node("image_proessing_node", anonymous=True, log_level=rospy.INFO)
    n = image_processing_node("image_proessing_node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down")