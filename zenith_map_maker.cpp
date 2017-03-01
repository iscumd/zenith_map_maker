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
#include <boost/foreach.hpp>

#include<zenith_obstacle_detector/Obstacle.h>
#include<zenith_obstacle_detector/ObstacleList.h>

ros::Publisher filter_pub, voxel_pub, ObsArray_pub;

  // All the objects needed
//  pcl::PCDReader reader;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg; 
//  pcl::PCDWriter writer;
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
  pass.setFilterLimits (-0.3, 1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredz);

  pass.setInputCloud (cloud_filteredz);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3, 3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredy);


  pass.setInputCloud (cloud_filteredy);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.3, 4);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filteredx);

  ROS_INFO("Point Cloud After Position Filter %d Points", cloud_filteredx->points.size());

  filter_pub.publish (cloud_filteredx);

  ROS_INFO("Published Position Filter Point Cloud");

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filteredx);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_final);

  ROS_INFO("Point Cloud After Voxel Filter %d Points", cloud_filtered_final->points.size());

  voxel_pub.publish (cloud_filtered_final);

  ROS_INFO("Published Voxel Filter Point Cloud");

//Start Ken clustering Segmentation

  tree->setInputCloud (cloud_filtered_final);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (400);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
 ec.setInputCloud (cloud_filtered_final);
  ec.extract (cluster_indices);

  zenith_obstacle_detector::ObstacleList obs_list;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (cloud_filtered_final->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pcl::PointXYZRGB max;
    pcl::PointXYZRGB min;
    pcl::getMinMax3D(*cloud_cluster, min, max);

    zenith_obstacle_detector::Obstacle obs; 

    float xcenter, ycenter, xsize, ysize, zsize;
    xcenter = (max.x + min.x)/2;
    ycenter = (max.y + min.y)/2;

    xsize = (max.x - min.x);
    ysize = (max.y - min.y);
    zsize = (max.z - min.z);

    obs.x = xcenter;
    obs.y = ycenter;
    
    if(ysize > .45 && ysize < 333){
       obs.type = "moving";
    }else if(ysize > .08 && ysize < .25){
       obs.type = "static";
    }else{
       obs.type = "unkown";
    }

    obs_list.obstacles.push_back(obs);

    ROS_INFO_STREAM("PointCloud representing the Cluster: " << j << " Has " << cloud_cluster->points.size () << " data points.");
    ROS_INFO_STREAM("Center x:" << xcenter << " y:" << ycenter << " z:" << (max.z + min.z)/2);
    ROS_INFO_STREAM("Size x:" << xsize << " y:" << ysize << " z:" << zsize);
    j++;
  }

  ObsArray_pub.publish(obs_list);

//END Ken clustering Segmentation


//Start Ken Planer Suface Segmentation Attempt
/*
    // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered_final);
  ne.setKSearch (10);
  ne.compute (*cloud_normals);
  ROS_INFO("Norms Computed");

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered_final);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  ROS_INFO_STREAM("Plane coefficients: " << *coefficients_plane << std::endl);
  ROS_INFO("Planner Segmentation Complete");
  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered_final);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  extract.filter (*cloud_plane);
  ROS_INFO("Planner Extraction Complete");
  ROS_INFO("PointCloud representing the planar component: %d data points.");
//  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  plane_pub.publish (cloud_plane);

  ROS_INFO("Planner Extraction Published");
*/
//END Ken Planer Segmentation
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_pcl_segmenter");
  ros::NodeHandle nh;
  filter_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_filtered_pointcloud", 1); 
  ObsArray_pub = nh.advertise<zenith_obstacle_detector::ObstacleList>("zenith/obstacles", 1);
  voxel_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("zed_voxel_pointcloud", 1);
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/zed/point_cloud/cloud_registered", 1, zed_pointcloud_callback);
  ros::spin();
}
