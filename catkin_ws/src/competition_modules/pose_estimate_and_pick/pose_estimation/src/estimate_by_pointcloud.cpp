#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
bool pose_estimate_cb(competition_msgs::pose_estimationRequest& req,competition_msgs::pose_estimationResponse& resp);

bool pose_estimate_cb(competition_msgs::pose_estimationRequest& req,
                      competition_msgs::pose_estimationResponse& resp)
{
  PointCloudXYZRGB::Ptr pcl(new PointCloudXYZRGB);
  pcl::fromROSMsg (req.pc_in, *pcl);
  printf("original Cloud Number: %d\n",pcl->points.size());

  PointCloudXYZRGB::Ptr filtered_pcl(new PointCloudXYZRGB);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl,*filtered_pcl,indices);
  printf("Nonnan Cloud Number: %d\n",filtered_pcl->points.size());
  resp.id = 123;
  return true;
}

int main (int argc, char** argv)
{
  //ros initialize ROS
  ros::init (argc,argv, "estimate_by_pointcloud");
  ros::NodeHandle nh;
  

  ros::ServiceServer service = nh.advertiseService("pose_estimate_byPC", pose_estimate_cb);
  ROS_INFO("Ready to extract pointcloud.");
  ros::spin();
  return 0;

}