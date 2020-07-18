#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from competition_msgs.srv import object_detection, object_detectionRequest, object_detectionResponse

mask_pub=rospy.Publisher('/object_detection/dataset1_mask', Image, queue_size=10)


def main():
    rospy.init_node('test_dataset1_client', anonymous=False)
    #rospy.Subscriber('/camera/color/image_raw', Image, maskCallback)
    rospy.spin()
    # ============= service setting =================== #
    # dataset1 object detect
    #try:
    #    rospy.wait_for_service('dataset1_object_detection', timeout=20) # rospy.wait_for_service('service name')
    #except (rospy.ServiceException, rospy.ROSException) as e:
    #    rospy.logerr("Service failed: %s"%e)
    #    exit(-1)
    #object_detect = rospy.ServiceProxy('dataset1_object_detection', object_detection) #rospy.ServiceProxy('service name',service class)

    # ============== wait for image & pointcloud message ================== #
    
    img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=20)
    rospy.sleep(3)
    #pc_msg = rospy.wait_for_message('/camera/depth_registered/points',PointCloud2, timeout=20)
    #mask_pub.publish(img_msg)
    # ============== dataset1 object detect  (call the service) ================== #
    #od_request = object_detectionRequest()
    #od_request.rgb_image = img_msg # img_msg as the object_dtection service's request
    #od_response = object_detect(od_request.rgb_image)
    #mask_pub.publish(od_response)
    # ============== dataset1 pose estimation ================ #

def maskCallback(msg):
    mask_pub.publish(msg)

if __name__=="__main__":
    main()
    
    




