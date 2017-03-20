#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>

int img_width = 500;
int img_height = 500;

float scan_width = 5;
float scan_depth = 5;

float step_width = scan_width/img_width;
float step_depth = scan_depth/img_height;

ros::Publisher filter_pub, voxel_pub, ObsArray_pub;

image_transport::ImageTransport it_;
image_transport::Publisher image_pub_;

  // All the objects needed
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//  pcl::ExtractIndices<pcl::Normal> extract_normals;

  // Datasets
//  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

void zed_pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& zed_point_cloud)
{
  ROS_INFO("Point Cloud Received With %d Points", zed_point_cloud->points.size());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_final (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
 
  pass.setInputCloud (zed_point_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1, -0.3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredz);

  pass.setInputCloud (cloud_filteredz);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-scan_width/2, scan_width/2);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredy);


  pass.setInputCloud (cloud_filteredy);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.01, scan_depth);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredx);

  ROS_INFO("Point Cloud After Position Filter %d Points", cloud_filteredx->points.size());

  filter_pub.publish (cloud_filteredx);

  ROS_INFO("Published Position Filter Point Cloud");

//Temporaryly removing sub sampling
//cloud_filtered_final = cloud_filteredx;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filteredx);
  sor.setLeafSize (step_depth, step_width, 10.0);
  sor.filter (*cloud_filtered_final);

  ROS_INFO("Point Cloud After Voxel Filter %d Points", cloud_filtered_final->points.size());

  voxel_pub.publish (cloud_filtered_final);

  ROS_INFO("Published Voxel Filter Point Cloud");


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_pcl_segmenter");
  ros::NodeHandle nh;
  filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_filtered_pointcloud", 1); 
  voxel_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_voxel_pointcloud", 1);
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/zed/point_cloud/cloud_registered", 1, zed_pointcloud_callback);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  ros::spin();
}
