#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main (int argc, char** argv)
{
    //ros initialize ROS
    ros::init (argc,argv, "estimate_by_pointcloud");
    ros::NodeHandle nh;
    //create a subscriber for the input point cloud
    //ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
    
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/object_detection/dataset1_mask", 1, imageCb);

    //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("mask", 1000);
    //ros::Rate loop_rate(10);
}