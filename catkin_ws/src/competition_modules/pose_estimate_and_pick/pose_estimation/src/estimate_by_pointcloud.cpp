#include <math.h>
#include <signal.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
// OpenCV_ROS bridge
#include <cv_bridge/cv_bridge.h>
#include <iostream>

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

//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

//msg &srv
#include <competition_msgs/pose_estimation.h> //pose_estimation.srv

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

#define classNum 5
static const int classID[classNum]={0, 100, 150, 200};

class EstimationNode{
  public:
    //function declaration
    EstimationNode(ros::NodeHandle nh_);
    static void sigint_cb(int sig);
    bool pose_estimate_cb(competition_msgs::pose_estimationRequest& req,competition_msgs::pose_estimationResponse& resp);
    void extract_pointcloud( cv_bridge::CvImagePtr mask_img,
                             PointCloudXYZRGB::Ptr cloud_in,
                             PointCloudXYZRGB::Ptr cloud_out );
    //geometry_msgs::pose pose_estimating_by_pca(PointCloudXYZRGB::Ptr object_pcl);


    //ros realted
    ros::NodeHandle nh;
    ros::ServiceServer service ;
    ros::Publisher ObjectExtract_pub ;
    
};

EstimationNode::EstimationNode(ros::NodeHandle nh_): nh(nh_){
  // Signal callback
  signal(SIGINT, sigint_cb); /// for shut down

  ObjectExtract_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_no_background", 1);
  //pcl_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  service = nh.advertiseService("pose_estimate_byPC", &EstimationNode::pose_estimate_cb, this);   ///??????????????????需要&與this
}



bool EstimationNode::pose_estimate_cb(competition_msgs::pose_estimationRequest& req,
                                      competition_msgs::pose_estimationResponse& resp)
{
  
  // ROS  image message to cv2
  cv_bridge::CvImagePtr mask_img = cv_bridge::toCvCopy(req.mask_img, sensor_msgs::image_encodings::TYPE_8UC1); 

  // tranform pointcloud from "camera_color_optical_frame" to "base_link"
  tf::TransformListener listener;
  tf::StampedTransform transform;//
  try{
	  listener.waitForTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), ros::Duration(20.0)); //  if you use Time(now) you should wait a few milliseconds for that information to arrive.
    listener.lookupTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), transform); //Time(0) means the latest available tranform in the buffer
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  sensor_msgs::PointCloud2 tmp_pcl_msg;
  pcl_ros::transformPointCloud("base_link", transform, (sensor_msgs::PointCloud2&)req.pc_in, tmp_pcl_msg);
  

  // ROS pointcloud message to PCL pointcloud ptr
  PointCloudXYZRGB::Ptr pcl_raw(new PointCloudXYZRGB);
  pcl::fromROSMsg (tmp_pcl_msg, *pcl_raw);
  
  // extract object's pointcloud by mask_img
  PointCloudXYZRGB::Ptr pcl_extracted(new PointCloudXYZRGB);
  EstimationNode::extract_pointcloud( mask_img, pcl_raw, pcl_extracted );
  
  

  /*
  PointCloudXYZRGB::Ptr filtered_pcl(new PointCloudXYZRGB);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl,*filtered_pcl,indices);
  printf("Nonnan Cloud Number: %d\n",filtered_pcl->points.size());
  */
  resp.id = 123;

  
  return true;
}


void EstimationNode::extract_pointcloud(  cv_bridge::CvImagePtr mask_img,
                                          PointCloudXYZRGB::Ptr cloud_in,
                                          PointCloudXYZRGB::Ptr cloud_out)
{
  // start to extract
  int img_height = mask_img->image.rows;
  int img_width = mask_img->image.cols;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  
  for (int id=1; id<classNum; id++){
    for (int y=0; y<img_height; y++){
      for (int x=0; x<img_width; x++){
        if (mask_img->image.at<uchar>(y,x) != 0){
          int index = y*img_width+x;
          inliers->indices.push_back(index);
        }
      }
    }
    PointCloudXYZRGB::Ptr cloud_extracted(new PointCloudXYZRGB);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);                
    extract.setNegative(false);  //false: 筛选Index对应的点，true：过滤获取Index之外的点               
    extract.filter(*cloud_extracted);
    printf("extract Cloud Number: %d\n",cloud_extracted->points.size());
    if(cloud_extracted->points.size() < 10)
      continue;

    //rempve NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_extracted,*cloud_extracted,indices);
    printf("Non NAN extract Cloud Number: %d\n",cloud_extracted->points.size());
    if(cloud_extracted->points.size() < 10)
      continue;

    //remove outliers
    PointCloudXYZRGB::Ptr cloud_extracted_filtered(new PointCloudXYZRGB);
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_extracted);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(5);
    outrem.filter(*cloud_extracted_filtered);//apply filter


    //pointcloud ptr to ROSmessage
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_extracted_filtered, cloud_msg);
    while(ros::ok()){
      cloud_msg.header.frame_id = "base_link";
      printf("frame_id: %s\n", cloud_msg.header.frame_id.c_str());
      ObjectExtract_pub.publish(cloud_msg);
    }
  }
  
}


void EstimationNode::sigint_cb(int sig) {
    printf("ROS Node: %s is shutdown.\n", ros::this_node::getName().c_str());
    //cout << "\nROS Node: " << ros::this_node::getName() << " is shutdown." << endl;
    ros::shutdown();
}


/*  
geometry_msgs::Pose EstimationNode::pose_estimating_by_pca(PointCloudXYZRGB::Ptr object_pcl)
{
  ////pca
  Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*pcl_extract, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*pcl_extract, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向間垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "質心點(4x1):\n" << pcaCentroid << std::endl;

  Eigen::Matrix4f tfmatrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tfmatrix_inv = Eigen::Matrix4f::Identity();
  tfmatrix.block<3, 3>(0, 0) = eigen_vectors.transpose();   //R.
  tfmatrix.block<3, 1>(0, 3) = -1.0f * (eigen_vectors.transpose()) *(centroid.head<3>());//  -R*t
  tfmatrix_inv = tfmatrix.inverse();

  // cout << "tfmatrix(4x4):\n" << tfmatrix << endl;
  // cout << "tfmatrix_inv(4x4):\n" << tfmatrix_inv << endl;
  geometry_msgs::Pose pca_pose_msg;
  pca_pose_msg.position.x = centroid[0];
  pca_pose_msg.position.y = centroid[1];
  pca_pose_msg.position.z = centroid[2];
  Eigen::Quaternionf q1(tfmatrix_inv.topLeftCorner<3, 3>());
  pca_pose_msg.orientation.x = q1.x();
  pca_pose_msg.orientation.y = q1.y();
  pca_pose_msg.orientation.z = q1.z();
  pca_pose_msg.orientation.w = q1.w();
  
}*/
  

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