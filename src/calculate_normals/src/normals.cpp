#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

#include <vector>
#include <angles/angles.h>
#include <limits>

class Calculate_Normal{
public:
  Calculate_Normal(){
    point_cloud_sub = nh.subscribe("/hokuyo3d/hokuyo_cloud", 1, &Calculate_Normal::normalCallBack, this);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 4000, 1);
    pub_obstacle = nh.advertise<sensor_msgs::PointCloud> ("/obstacle_cloud", 600, 1);
    pub_normals = nh.advertise<pcl::PointCloud<pcl::PointNormal> > ("/normals", 5000);
    ros::Rate loop_rate(10);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub;
  ros::Publisher pub_filtered;
  ros::Publisher pub_obstacle;
  ros::Publisher pub_normals;
  sensor_msgs::PointCloud d_cloud;
  sensor_msgs::PointCloud base_cloud;
  sensor_msgs::PointCloud obstacle;
  sensor_msgs::PointCloud2 in_cloud2;
  sensor_msgs::PointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 obstacle_cloud2;
  tf::TransformListener listener;

void normalCallBack (const sensor_msgs::PointCloudConstPtr& in_cloud1)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr y_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

  d_cloud = *in_cloud1;
  //transform
  try{
    listener.waitForTransform("/base_link", d_cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
    listener.transformPointCloud("/base_link", d_cloud.header.stamp, d_cloud, d_cloud.header.frame_id, base_cloud);
  }catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  sensor_msgs::convertPointCloudToPointCloud2 (base_cloud, in_cloud2);

  pcl::fromROSMsg (in_cloud2, *pcl_cloud);

  //passthrough
  pcl::PassThrough<pcl::PointXYZ> z_pass;
  z_pass.setInputCloud (pcl_cloud);
  z_pass.setFilterFieldName("z");
  z_pass.setFilterLimits (-1.0, 1.0);
  z_pass.filter (*z_passthrough_cloud);

  pcl::PassThrough<pcl::PointXYZ> y_pass;
  y_pass.setInputCloud (z_passthrough_cloud);
  y_pass.setFilterFieldName("y");
  y_pass.setFilterLimits (-10.0, 10.0);
  y_pass.filter (*y_passthrough_cloud);

  pcl::PassThrough<pcl::PointXYZ> x_pass;
  x_pass.setInputCloud (y_passthrough_cloud);
  x_pass.setFilterFieldName("x");
  x_pass.setFilterLimits (0.0, 10.0);
  x_pass.filter (*passthrough_cloud);

  //estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud(passthrough_cloud);
  ne.setKSearch (20);
  ne.compute(*normals);

  copyPointCloud(*passthrough_cloud, *obstacle_cloud);

  float threshold = 1.57-0.244;
  float NaN = std::numeric_limits<float>::quiet_NaN();

  //calculate normal vectors
  for(size_t i = 0; i<normals->points.size(); ++i)
  {
    normals->points[i].x = passthrough_cloud->points[i].x;
    normals->points[i].y = passthrough_cloud->points[i].y;
    normals->points[i].z = passthrough_cloud->points[i].z;

    tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
    tf::Vector3 up_vector(0.0, 0.0, 1.0);

    //cross(外積) dot(内積) normalize(正規化)
    tf::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();
    float rad = q.getAngle();
    float cuv;
    if(rad < 1.57){
      cuv = 1.57 - rad;
    }else{
      cuv = rad - 1.57;
    }
    normals->points[i].curvature = cuv;

    if(cuv < threshold)
    {
      passthrough_cloud->points[i].x = NaN;
      passthrough_cloud->points[i].y = NaN;
      passthrough_cloud->points[i].z = NaN;
      obstacle_cloud->points[i].x = normals->points[i].x;
      obstacle_cloud->points[i].y = normals->points[i].y;
      obstacle_cloud->points[i].z = normals->points[i].z;
    }else{
      passthrough_cloud->points[i].x = normals->points[i].x;
      passthrough_cloud->points[i].y = normals->points[i].y;
      passthrough_cloud->points[i].z = normals->points[i].z;
      obstacle_cloud->points[i].x = NaN;
      obstacle_cloud->points[i].y = NaN;
      obstacle_cloud->points[i].z = NaN;
    }
  }

  pcl::toROSMsg (*passthrough_cloud, cloud_filtered);
  pcl::toROSMsg (*obstacle_cloud, obstacle_cloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(obstacle_cloud2, obstacle);

  // Publish the data
  pub_filtered.publish (cloud_filtered);
  pub_obstacle.publish (obstacle);
  pub_normals.publish (normals);

  int k;
  k=obstacle.points.size();
  ROS_INFO("obstacle size: %d\n", k);
}
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "normal_filter");
  Calculate_Normal calculate_normal;
  calculate_normal;

  ros::spin();
}
