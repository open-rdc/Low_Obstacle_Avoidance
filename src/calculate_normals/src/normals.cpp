#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <angles/angles.h>
#include <limits>
#include <laser_assembler/AssembleScans.h>

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher poseArrayPub;
geometry_msgs::PoseArray poseArray; // particles as PoseArray (preallocated)

void normalCallback (const sensor_msgs::PointCloudConstPtr& in_cloud1)
{
  tf::TransformListener listener;
  tf::StampedTransform transform1;
  try{
    listener.waitForTransform("/base_link", "/hokuyo3d_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/base_link", "/hokuyo3d_link", ros::Time(0), transform1);
  }catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Matrix3x3 mat = transform1.getBasis();
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  double threshold = 0.392;
  double thresholdM = 1.57 - pitch - threshold;
  double thresholdP = 1.57 + pitch + threshold;

  sensor_msgs::PointCloud2 in_cloud2;
  sensor_msgs::convertPointCloudToPointCloud2 (*in_cloud1, in_cloud2);
  sensor_msgs::PointCloud2 cloud2_filtered;
  sensor_msgs::PointCloud2 obstacle;

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr y_passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr ror_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  float NaN = std::numeric_limits<float>::quiet_NaN();

  pcl::fromROSMsg (in_cloud2, *pcl_cloud);

  poseArray.poses.clear();
  poseArray.header.stamp = ros::Time(0);
  poseArray.header.frame_id = pcl_cloud->header.frame_id; //EDITED
  ROS_INFO_STREAM("poseArray.header: frame=" << poseArray.header.frame_id);

  //passthrough
  pcl::PassThrough<pcl::PointXYZ> z_pass;
  z_pass.setInputCloud (pcl_cloud);
  z_pass.setFilterFieldName("z");
  z_pass.setFilterLimits (-1.0, 0.4);
  z_pass.filter (*z_passthrough_cloud);

  pcl::PassThrough<pcl::PointXYZ> y_pass;
  y_pass.setInputCloud (z_passthrough_cloud);
  y_pass.setFilterFieldName("y");
  y_pass.setFilterLimits (-8.0, 8.0);
  y_pass.filter (*y_passthrough_cloud);

  pcl::PassThrough<pcl::PointXYZ> x_pass;
  x_pass.setInputCloud (y_passthrough_cloud);
  x_pass.setFilterFieldName("x");
  x_pass.setFilterLimits (0.0, 10.0);
  x_pass.filter (*passthrough_cloud);

  //outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(passthrough_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*sor_passthrough_cloud); 

  //voxel_grid
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (sor_passthrough_cloud);
  vg.setLeafSize (0.05, 0.05, 0.05);
  vg.filter (*voxel_cloud);

  // estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud(voxel_cloud);
  ne.setKSearch (24);
  pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
  ne.compute(*normals);

  copyPointCloud(*voxel_cloud, *obstacle_cloud);

  //publish normal vectors
  for(size_t i = 0; i<normals->points.size(); ++i)
  {
    normals->points[i].x = voxel_cloud->points[i].x;
    normals->points[i].y = voxel_cloud->points[i].y;
    normals->points[i].z = voxel_cloud->points[i].z;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Quaternion msg;

    tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
    tf::Vector3 up_vector(1.0, 0.0, 0.0);

    //cross(外積)　dot(内積) normalize(正規化)
    tf::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    double angle = -1.0*acos(axis_vector.dot(up_vector));
    tf::Quaternion q(right_vector, angle);
    q.normalize();
    double rad = q.getAngle();
    
    tf::quaternionTFToMsg(q, msg);
    pose.pose.position.x = normals->points[i].x;
    pose.pose.position.y = normals->points[i].y;
    pose.pose.position.z = normals->points[i].z;
    pose.pose.orientation = msg;
    poseArray.poses.push_back(pose.pose);

    //if(!((1.57-0.0789) < rad && rad < (1.65+0.0789)))
    //if(!((1.57-0.785) < rad && rad <(1.57+0.78)))
    //if(!((1.76-0.0798) < rad && rad < (1.76+0.0798)))
    //if(!((1.76-0.392) < rad && rad < (1.76+0.392)))
    if(!(thresholdM < rad && rad < thresholdP))
    {
      //ROS_INFO("get_angle: %lf\n", rad);
      voxel_cloud->points[i].x = NaN;
      voxel_cloud->points[i].y = NaN;
      voxel_cloud->points[i].z = NaN;
      obstacle_cloud->points[i].x = normals->points[i].x;
      obstacle_cloud->points[i].y = normals->points[i].y;
      obstacle_cloud->points[i].z = normals->points[i].z;
    }else{
      voxel_cloud->points[i].x = normals->points[i].x;
      voxel_cloud->points[i].y = normals->points[i].y;
      voxel_cloud->points[i].z = normals->points[i].z;
      obstacle_cloud->points[i].x = NaN;
      obstacle_cloud->points[i].y = NaN;
      obstacle_cloud->points[i].z = NaN;
    }
  }
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud( *obstacle_cloud, *obstacle_cloud, indices);
  pcl::PassThrough<pcl::PointXYZ> z_pass2;
  z_pass2.setInputCloud (obstacle_cloud);
  z_pass2.setFilterFieldName("z");
  z_pass2.setFilterLimits (-1.0, 0.4);
  z_pass2.filter (*z_passthrough_cloud2);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud (z_passthrough_cloud2);
  outrem.setRadiusSearch (0.1);
  outrem.setMinNeighborsInRadius (12);
  outrem.filter (*ror_cloud);

  poseArrayPub.publish(poseArray);

  pcl::toROSMsg (*voxel_cloud, cloud2_filtered);
  pcl::toROSMsg (*ror_cloud, obstacle);

  // Publish the data
  pub.publish (cloud2_filtered);
  pub2.publish (obstacle);

  int j;
  j=poseArray.poses.size();
  ROS_INFO("poseArray size: %d\n", j);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "normal_filter");
  ros::NodeHandle nh;

  ros::Rate loop_rate(5.0);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/hokuyo3d/hokuyo_cloud", 1, normalCallback);
  //ros::Subscriber sub = nh.subscribe ("/cloud_pcd", 2, normalCallback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 2000, 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_cloud", 2000, 1);
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/normal_vectors", 5000, 1);

  // Spin
  ros::spin();
}
