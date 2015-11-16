#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

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
#define allowable_error 0.5

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher low_obstacle_pub;
sensor_msgs::PointCloud obstacle;

class Calculate_Normal{
public:
  Calculate_Normal(){
    point_cloud_sub = nh.subscribe("/hokuyo3d/hokuyo_cloud", 1, &Calculate_Normal::normalCallBack, this);
    scan_sub = nh.subscribe("/scan", 1, &Calculate_Normal::scanCallBack, this);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 4000, 1);
    low_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2> ("/low_obstacle_cloud", 600, 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud> ("/obstacle_cloud", 600, 1);
    pub3 = nh.advertise<pcl::PointCloud<pcl::PointNormal> > ("/normals", 5000);
    ros::Rate loop_rate(30);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub;
  ros::Subscriber scan_sub;
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher low_obstacle_pub;
  sensor_msgs::PointCloud d_cloud;
  sensor_msgs::PointCloud base_cloud;
  sensor_msgs::PointCloud2 in_cloud2;
  sensor_msgs::PointCloud2 cloud2_filtered;
  sensor_msgs::PointCloud2 obstacle2;
  sensor_msgs::PointCloud scan_cloud;
  sensor_msgs::PointCloud d_scan_cloud;
  sensor_msgs::PointCloud low_obstacle;
  sensor_msgs::PointCloud passthrough_scan_cloud;
  sensor_msgs::PointCloud2 passthrough_scan_cloud2;
  sensor_msgs::PointCloud2 scan_cloud2;
  sensor_msgs::PointCloud2 normals;
  tf::TransformListener listener;
  tf::TransformListener scan_listener;

  void normalCallBack (const sensor_msgs::PointCloudConstPtr& in_cloud1)
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud2(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr y_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr sor_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_passthrough(new pcl::PointCloud<pcl::PointXYZ>);   
      pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

      d_cloud = *in_cloud1;
      try{
          listener.waitForTransform("/base_link", d_cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
          //listener.lookupTransform("/base_link", "/hokuyo3d", ros::Time(0), transform1);
          listener.transformPointCloud("/base_link", base_cloud.header.stamp, d_cloud, d_cloud.header.frame_id, base_cloud);
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
      //
      // //outlier removal
      // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      // sor.setInputCloud(passthrough_cloud);
      // sor.setMeanK(50);
      // sor.setStddevMulThresh(1.0);
      // sor.filter(*sor_passthrough_cloud); 

      //voxel_grid
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud (passthrough_cloud);
      vg.setLeafSize (0.05, 0.05, 0.05);
      vg.filter (*voxel_cloud);

      // estimate normals
      pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
      ne.setInputCloud(voxel_cloud);
      ne.setKSearch (24);
      ne.compute(*normals);

      copyPointCloud(*voxel_cloud, *obstacle_cloud);

      float threshold = 0.392;
      float thresholdM = 1.57  - threshold;
      float thresholdP = 1.57  + threshold;
      float NaN = std::numeric_limits<float>::quiet_NaN();
      //publish normal vectors
      for(size_t i = 0; i<normals->points.size(); ++i)
      {
          normals->points[i].x = voxel_cloud->points[i].x;
          normals->points[i].y = voxel_cloud->points[i].y;
          normals->points[i].z = voxel_cloud->points[i].z;

          tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
          tf::Vector3 up_vector(0.0, 0.0, 1.0);

          //cross(外積)　dot(内積) normalize(正規化)
          tf::Vector3 right_vector = axis_vector.cross(up_vector);
          right_vector.normalized();
          tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
          q.normalize();
          float rad = q.getAngle();
          // float axis = q.getAxis();

          // tf::Matrix3x3 mat(q);
          // double roll, pitch, yaw;
          // mat.getRPY(roll, pitch, yaw);

          float cuv;
          if(rad < 1.57){
              cuv = 1.57 - rad;
          }else{
              cuv = rad - 1.57;
          }
          normals->points[i].curvature = cuv;

          //if(!((1.57-0.0789) < rad && rad < (1.65+0.0789)))
          //if(!((1.57-0.785) < rad && rad <(1.57+0.78)))
          //if(!((1.76-0.0798) < rad && rad < (1.76+0.0798)))
          // if(!((1.76-0.392) < rad && rad < (1.76+0.392)))
          // if(!((thresholdM) < rad && rad < (thresholdP)))
          // if(!((thresholdM) < pitch && pitch < (thresholdP)))
          if(!(cuv < threshold))
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
      pcl::PassThrough<pcl::PointXYZ> z_pass2;
      z_pass2.setInputCloud (voxel_cloud);
      z_pass2.setFilterFieldName("z");
      z_pass2.setFilterLimits (-1.0, 0.2);
      z_pass2.filter (*z_passthrough_cloud2);

      pcl::PassThrough<pcl::PointXYZ> obstacle_pass;
      obstacle_pass.setInputCloud (obstacle_cloud);
      obstacle_pass.setFilterFieldName("z");
      obstacle_pass.setFilterLimits (-1.0, 0.2);
      obstacle_pass.filter (*obstacle_passthrough);

      pcl::toROSMsg (*z_passthrough_cloud2, cloud2_filtered);
      pcl::toROSMsg (*obstacle_passthrough, obstacle2);
      sensor_msgs::convertPointCloud2ToPointCloud(obstacle2, obstacle);

      // Publish the data
      pub.publish (cloud2_filtered);
      pub2.publish (obstacle);
      pub3.publish (normals);

      int k;
      k=obstacle.points.size();
      ROS_INFO("obstacle size: %d\n", k);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
      float distance;
      int i, j, count, k = 0, low_obstacle_num = 0;
      sensor_msgs::PointCloud2 low_obstacle_filtered;
      sensor_msgs::PointCloud2 low_obstacle2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr y_passthrough_scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr x_passthrough_scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_low_obstacle (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_low_obstacle_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      laser_geometry::LaserProjection projector;
      projector.projectLaser(*scan, d_scan_cloud);

      // float scan_cloud_x[(int)scan_cloud.points.size()], scan_cloud_y[(int)scan_cloud.points.size()], scan_cloud_intensity[(int)scan_cloud.channels[0].values.size()];
      // count = 0;
      // for(i = 0; i < (int)scan_cloud.points.size(); i++){
      //   if((float)hypotf(scan_cloud.points[i].x, scan_cloud.points[i].y) < 29){
      //     scan_cloud_x[count] = scan_cloud.points[i].x;
      //     scan_cloud_y[count] = scan_cloud.points[i].y;
      //     scan_cloud_intensity[count] = scan_cloud.channels[0].values[i];
      //     count += 1;
      //   }
      // }
      // scan_cloud.points.resize(count);
      // scan_cloud.channels[0].values.resize(count);
      // for(i = 0; i < count; i++){
      //   scan_cloud.points[i].x = scan_cloud_x[i];
      //   scan_cloud.points[i].y = scan_cloud_y[i];
      //   scan_cloud.channels[0].values[i] = scan_cloud_intensity[i];
      // }
      ros::Time current_time;
      current_time = ros::Time::now();
      geometry_msgs::TransformStamped base_trans;
      base_trans.header.frame_id = "/base_link";
      base_trans.header.stamp = current_time;
      ros::Time past_time;
      past_time = obstacle.header.stamp;
      try{
          scan_listener.waitForTransform("/base_link", d_scan_cloud.header.frame_id, d_scan_cloud.header.stamp, ros::Duration(1.0));
          scan_listener.transformPointCloud("/base_link", past_time, d_scan_cloud, d_scan_cloud.header.frame_id, scan_cloud);
      }catch(tf::TransformException &ex){
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
      }

      sensor_msgs::convertPointCloudToPointCloud2(scan_cloud, scan_cloud2);
      pcl::fromROSMsg(scan_cloud2, *pcl_scan_cloud);

      pcl::PassThrough<pcl::PointXYZ> y_pass_scan;
      y_pass_scan.setInputCloud (pcl_scan_cloud);
      y_pass_scan.setFilterFieldName("y");
      y_pass_scan.setFilterLimits (-10.0, 10.0);
      y_pass_scan.filter (*y_passthrough_scan_cloud);

      pcl::PassThrough<pcl::PointXYZ> x_pass_scan;
      x_pass_scan.setInputCloud (y_passthrough_scan_cloud);
      x_pass_scan.setFilterFieldName("x");
      x_pass_scan.setFilterLimits (0.0, 10.0);
      x_pass_scan.filter (*x_passthrough_scan_cloud);

      pcl::VoxelGrid<pcl::PointXYZ> vogl;
      vogl.setInputCloud (x_passthrough_scan_cloud);
      vogl.setLeafSize (0.05, 0.05, 0.05);
      vogl.filter (*voxel_scan_cloud);

      pcl::toROSMsg(*voxel_scan_cloud, passthrough_scan_cloud2);
      sensor_msgs::convertPointCloud2ToPointCloud(passthrough_scan_cloud2, passthrough_scan_cloud);

      for(i = 0; i < (int)obstacle.points.size(); i++){
          count = 0;
          // ROS_INFO("obstacle x = %f, y = %f ", obstacle.points[i].x, obstacle.points[i].y);
          for(j = 0; j < (int)passthrough_scan_cloud.points.size(); j++){
              distance = hypotf(passthrough_scan_cloud.points[j].x - obstacle.points[i].x, passthrough_scan_cloud.points[j].y - obstacle.points[i].y);
              if(distance > allowable_error){
                  count += 1;
                  // ROS_INFO("scan x = %f, y = %f", passthrough_scan_cloud.points[j].x, passthrough_scan_cloud.points[j].y);
              }
          }
          if(count == (int)passthrough_scan_cloud.points.size()){
              low_obstacle_num += 1;
          }
      }

      low_obstacle.points.resize(low_obstacle_num);
      low_obstacle.channels.resize(1);
      low_obstacle.channels[0].name = "intensity";
      low_obstacle.channels[0].values.resize(low_obstacle_num);

      for(i = 0; i < (int)obstacle.points.size(); i++){
          count = 0;
          for(j = 0; j< (int)passthrough_scan_cloud.points.size(); j++){
              distance = hypotf(passthrough_scan_cloud.points[j].x - obstacle.points[i].x, passthrough_scan_cloud.points[j].y - obstacle.points[i].y);
              if(distance > allowable_error){
                  count += 1;
              }
          }
          if(count == (int)passthrough_scan_cloud.points.size()){
              low_obstacle.points[k].x = obstacle.points[i].x;
              low_obstacle.points[k].y = obstacle.points[i].y;
              low_obstacle.points[k].z = obstacle.points[i].z;
              k += 1;
          }
      }

      low_obstacle.header.stamp = obstacle.header.stamp;
      low_obstacle.header.frame_id = "/base_link";

      sensor_msgs::convertPointCloudToPointCloud2(low_obstacle, low_obstacle2);
      pcl::fromROSMsg(low_obstacle2, *pcl_low_obstacle);

      pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
      ror.setInputCloud(pcl_low_obstacle);
      ror.setRadiusSearch(0.5);
      ror.setMinNeighborsInRadius(50);
      ror.filter(*pcl_low_obstacle_filtered);

      pcl::toROSMsg(*pcl_low_obstacle_filtered, low_obstacle_filtered);

      low_obstacle_pub.publish(low_obstacle_filtered);
      // int l=low_obstacle_filtered.size();
      // ROS_INFO("low_obstacle_filtered size: %d\n", l);
      // low_obstacle_filtered.clear();
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "normal_filter");
  Calculate_Normal calculate_normal;
  calculate_normal;

  ros::spin();
}
