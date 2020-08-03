#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
// OpenCV_ROS bridge
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h> // PCA
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/filters/radius_outlier_removal.h> // RemoveOutlier
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h> // Passthrough
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
//msg &srv
#include <competition_msgs/pose_estimation.h> //pose_estimation.srv



typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;


class EstimationNode{
  public:
    //function declaration
    EstimationNode(ros::NodeHandle nh_);
    bool pose_estimate_cb(competition_msgs::pose_estimationRequest& req,competition_msgs::pose_estimationResponse& resp);

    //ros realted
    ros::NodeHandle nh;
    ros::ServiceServer service ;
    ros::Publisher ObjectExtract_pub ;
    
};

EstimationNode::EstimationNode(ros::NodeHandle nh_): nh(nh_){
  ObjectExtract_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_no_background", 1);
  //pcl_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  service = nh.advertiseService("pose_estimate_byPC", &EstimationNode::pose_estimate_cb, this);   ///??????????????????需要&與this
}



bool EstimationNode::pose_estimate_cb(competition_msgs::pose_estimationRequest& req,
                                      competition_msgs::pose_estimationResponse& resp)
{
  // ROS pointcloud message to PCL pointcloud ptr
  PointCloudXYZRGB::Ptr pcl_in(new PointCloudXYZRGB);
  pcl::fromROSMsg (req.pc_in, *pcl_in);
  printf("original Cloud Number: %d\n",pcl_in->points.size());
  // ROS  image message to cv2
  cv_bridge::CvImagePtr mask_img = cv_bridge::toCvCopy(req.mask_img, 
                                                          sensor_msgs::image_encodings::TYPE_8UC1); 
  
  
  
  // start to extract
  int img_height = mask_img->image.rows;
  int img_width = mask_img->image.cols;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  PointCloudXYZRGB::Ptr pcl_extract(new PointCloudXYZRGB);

  for (int y=0; y<img_height; y++){
    for (int x=0; x<img_width; x++){
      if (mask_img->image.at<uchar>(y,x) != 0){
        int index = y*img_width+x;
        inliers->indices.push_back(index);
      }
    }
  }

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pcl_in);
  extract.setIndices(inliers);                
  extract.setNegative(false);  //false: 筛选Index对应的点，true：过滤获取Index之外的点               
  extract.filter(*pcl_extract);
  printf("extract Cloud Number: %d\n",pcl_extract->points.size());
  //pointcloud ptr to ROSmessage
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*pcl_extract, cloud_msg);
  while(ros::ok()){
    ObjectExtract_pub.publish(cloud_msg);
  }
  
  /*
  PointCloudXYZRGB::Ptr filtered_pcl(new PointCloudXYZRGB);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl,*filtered_pcl,indices);
  printf("Nonnan Cloud Number: %d\n",filtered_pcl->points.size());
  */
  resp.id = 123;

  
  return true;
}

int main (int argc, char** argv)
{
  //ros initialize ROS
  ros::init (argc,argv, "estimate_by_pointcloud");
  ros::NodeHandle nh;
  EstimationNode node(nh);
  //ros::ServiceServer service = nh.advertiseService("pose_estimate_byPC", pose_estimate_cb);

  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  //ros::Rate loop_rate(10);


  ROS_INFO("Ready to extract pointcloud.");
  ros::spin();
  return 0;

}