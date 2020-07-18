#!/usr/bin/env python
# Yeh Tsai Yuan 2020/07/11

import rospy
import cv2 
import numpy as np;
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from competition_msgs.msg import object_detection, object_detectionResponse

cvbridge = CvBridge()
#image_pub=rospy.Publisher('/object_detection/dataset1_mask', Image, queue_size=10)

def main():

    rospy.init_node('dataset1_object_detection_node')
    rospy.Service('dataset1_object_detection',object_detection,dataset1_object_detect)
    rospy.spin()

def dataset1_object_detect(req):
    try:
        cv_img = cvbridge.imgmsg_to_cv2(req.rgb_img,"bgr8")
    except CvBridgeError as e:
        print(e)
    hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # setting threshold for show the red  
    lower_red_1 = np.array([170, 50, 50])
    upper_red_1 = np.array([180, 255,255])
    lower_red_2 = np.array([0, 50, 50])
    upper_red_2 = np.array([10, 255,255])
    mask_red_1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1) 
    mask_red_2 = cv2.inRange(hsv_img, lower_red_2, upper_red_2)
    mask_red = mask_red_1 + mask_red_2
    mask_red_eroded = cv2.erode(mask_red, None, iterations = 3)
    mask_red_eroded_dilated = cv2.dilate(mask_red_eroded, None, iterations =3)

    # setting threshold for show the green  
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([99, 255,255])
    mask_green = cv2.inRange(hsv_img, lower_green, upper_green) 
    mask_green_eroded = cv2.erode(mask_green, None, iterations = 3)
    mask_green_eroded_dilated = cv2.dilate(mask_green_eroded, None, iterations =3)

    # setting threshold for show the blue  
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([124, 255,255])
    mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue) 
    mask_blue_eroded = cv2.erode(mask_blue, None, iterations = 3)
    mask_blue_eroded_dilated = cv2.dilate(mask_blue_eroded, None, iterations =3)
    

    # find red object contour and draw a mask
    _, red_contours, _ = cv2.findContours(mask_red_eroded_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in red_contours:
        cnt_len = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02*cnt_len, True) # prefer 1%~5% contour length 
        #print(len(approx))
        cv2.drawContours(mask_red_eroded_dilated, [cnt], -1, 200, -1) 
        #(draw img, contour, -1, grayscale or rgb[255, 0, 0],-1=fullfilled)
    
    _, green_contours, _ = cv2.findContours(mask_green_eroded_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in green_contours:
        cnt_len = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.01*cnt_len, True)
        #print(len(approx))
        cv2.drawContours(mask_green_eroded_dilated, [cnt], -1, 150, -1) 
        #(draw img, contour, -1, grayscale or rgb[255, 0, 0],-1=fullfilled)
    
    _, blue_contours, _ = cv2.findContours(mask_blue_eroded_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in blue_contours:
        cnt_len = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
        #print(len(approx))
        cv2.drawContours(mask_blue_eroded_dilated, [cnt], -1, 100, -1) 
        #(draw img, contour, -1, grayscale or rgb[255, 0, 0],-1=fullfilled)

    
    print("publish dataset1 mask img")    
    mask = mask_blue_eroded_dilated + mask_red_eroded_dilated + mask_green_eroded_dilated
    img_msg = cvbridge.cv2_to_imgmsg(mask)
    #image_pub.publish(img_msg)
    return object_detectionResponse(img_msg)





if __name__=='__main__':
    main()
