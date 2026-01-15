#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
class ImageProcessor:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()

        self.mirrored = None
        masked_image = None
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

    def crop(self, img):

        cropped_img = img[200:500, 0:640]
        ros_cropped = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
        return cropped_img

    def hsv_image(self, cv_img):
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        return hsv_img
    
    def white_mask(self, hsv_img):
        mask_w = cv2.inRange(hsv_img, (0,0,180),(255,60,255))

        #Separates the white parts from original image
        white_part = cv2.bitwise_and(hsv_img,hsv_img,mask=mask_w)
        white_part = cv2.erode(white_part, self.kernel)
        white_part = cv2.dilate(white_part, self.kernel)
        white_part = cv2.dilate(white_part, self.kernel)
        white_part = cv2.dilate(white_part, self.kernel)
        white_part = cv2.dilate(white_part, self.kernel)
        bgr_white = cv2.cvtColor(white_part, cv2.COLOR_HSV2BGR)
        return bgr_white

    def yellow_mask(self, hsv_img):
        mask_y = cv2.inRange(hsv_img, (15,18,130),(35,255,255))

        #Separates the white parts from original image
        yellow_part = cv2.bitwise_and(hsv_img,hsv_img,mask=mask_y)
        yellow_part = cv2.erode(yellow_part, self.kernel)
        yellow_part = cv2.dilate(yellow_part, self.kernel)
        yellow_part = cv2.dilate(yellow_part, self.kernel)
        yellow_part = cv2.dilate(yellow_part, self.kernel)
        yellow_part = cv2.dilate(yellow_part, self.kernel)
        bgr_yellow = cv2.cvtColor(yellow_part, cv2.COLOR_HSV2BGR)
        return bgr_yellow

    def process(self, img):
        cropped_img = self.crop(img)
        hsv_cropped_img = self.hsv_image(cropped_img)

        masked_white = self.white_mask(hsv_cropped_img)
        masked_yellow = self.yellow_mask(hsv_cropped_img)
        masked_combined = cv2.bitwise_or(masked_white, masked_yellow)

        return cropped_img, masked_white, masked_yellow


class LaneDetection:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()

        # Subscribe to compressed camera img
        rospy.Subscriber("/ee483mm11/camera_node/image/compressed", CompressedImage, self.get_image, queue_size = 1, buff_size = 1000000)

        # Instantiate an image processor
        self.processor = ImageProcessor()

        self.lines_pub = rospy.Publisher("image_lines", Image, queue_size=10)

        # Initialize current image and a flag to start processing the images
        self.current_img = None
        self.img_flag = False

        # Initialize canny edge detection parameters
        self.canny_low = 90
        self.canny_high = 150

    def get_image(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.current_img = cv_img

        self.img_flag = True

    def detect_lanes(self):

        if self.img_flag:
            cv_cropped, cv_white, cv_yellow = self.processor.process(self.current_img)
            image_edges = self.canny(cv_cropped)
            white_edges = self.filter_lane_edges(cv_white, image_edges)
            yellow_edges = self.filter_lane_edges(cv_yellow, image_edges)

            output = np.copy(cv_cropped)
            output = self.hough_transform(white_edges, output, line_color=(255,0,0))
            output = self.hough_transform(yellow_edges, output, line_color=(0,255,0))

            img_msg = self.bridge.cv2_to_imgmsg(output, "bgr8")
            self.lines_pub.publish(img_msg)


    def canny(self, img):

        #get edges
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_edges = cv2.Canny(gray_image, self.canny_low, self.canny_high)
        return image_edges

    def filter_lane_edges(self, lane_img, img_edges):

        masked = cv2.bitwise_and(lane_img, lane_img, mask = img_edges)
        masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)      
        return masked


    def hough_transform(self, edges_img, overlay_img, line_color=(255,0,0)):

        #get lines from image
        lines = cv2.HoughLinesP(edges_img,1,np.pi/180,25,minLineLength=20,maxLineGap=20)

        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(overlay_img, (l[0],l[1]), (l[2],l[3]), line_color, 2, cv2.LINE_AA)
                cv2.circle(overlay_img, (l[0],l[1]), 2, line_color,-1)
                cv2.circle(overlay_img, (l[2],l[3]), 2, line_color,-1)

        return overlay_img
        
if __name__=="__main__":
    rospy.init_node("lane_detection", anonymous=True)
    rate = rospy.Rate(5)
    l = LaneDetection()
    rospy.sleep(5)
    while not rospy.is_shutdown():
        l.detect_lanes()
        rate.sleep()
    rospy.spin()