#!/usr/bin/env python
# Yeh Tsai Yuan 2020/07/11

import rospy
import cv2 
import numpy as np;
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
cvbridge = CvBridge()
rospy.init_node('object_detection',anonymous=False)
image_pub=rospy.Publisher('/object_detection/original_image', Image, queue_size=10)

def main():
    rospy.Subscriber('/camera/color/image_raw', Image, maskCallback)
    rospy.spin()

def maskCallback(msg):
    try:
        cv_img = cvbridge.imgmsg_to_cv2(msg,"bgr8")
    except CvBridgeError as e:
        print(e)
    hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    lower_red_1 = np.array([170, 50, 50])
    upper_red_1 = np.array([180, 255,255])
    lower_red_2 = np.array([0, 50, 50])
    upper_red_2 = np.array([10, 255,255])
    mask_red_1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1) 
    mask_red_2 = cv2.inRange(hsv_img, lower_red_2, upper_red_2)
    mask_red = mask_red_1 + mask_red_2
    mask_red_eroded = cv2.erode(mask_red, None, iterations = 3)
    mask_red_eroded_dilated = cv2.dilate(mask_red_eroded, None, iterations =3)
    
    _, contours, hierarchy = cv2.findContours(
        mask_red_eroded_dilated,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours[0]))
    print(len(contours[1]))
    cv2.drawContours(mask_red_eroded_dilated, contours, -1, (0,255,0), 3)
    img_msg = cvbridge.cv2_to_imgmsg(mask_red_eroded_dilated)
    image_pub.publish(img_msg)





if __name__=='__main__':
    main()
