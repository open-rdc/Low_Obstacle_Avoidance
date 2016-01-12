#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
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

#include <vector>
#include <angles/angles.h>
#include <limits>

class Calculate_Normal{
public:
    Calculate_Normal(){
    point_cloud_sub = nh.subscribe("/hokuyo3d/hokuyo_cloud", 1, &Calculate_Normal::normalCallBack, this);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud2", 4000, 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud> ("/obstacle_cloud2", 600, 1);
    pub3 = nh.advertise<pcl::PointCloud<pcl::PointNormal> > ("/normals2", 5000);
    ros::Rate loop_rate(10);
  }

private:
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    sensor_msgs::PointCloud d_cloud;
    sensor_msgs::PointCloud base_cloud;
    sensor_msgs::PointCloud obstacle;
    sensor_msgs::PointCloud2 in_cloud2;
    sensor_msgs::PointCloud2 cloud_filtered;
    sensor_msgs::PointCloud2 obstacle2;
    tf::TransformListener listener;

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
    x_pass.setFilterLimits (0.30, 10.0);
    x_pass.filter (*passthrough_cloud);

    //outlier removal
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
    ne.setKSearch (20);
    ne.compute(*normals);

    copyPointCloud(*voxel_cloud, *obstacle_cloud);

    float threshold = 1.57-0.244;
    float NaN = std::numeric_limits<float>::quiet_NaN();
    //publish normal vectors
    for(size_t i = 0; i<normals->points.size(); ++i)
    {
        normals->points[i].x = voxel_cloud->points[i].x;
        normals->points[i].y = voxel_cloud->points[i].y;
        normals->points[i].z = voxel_cloud->points[i].z;

        tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
        tf::Vector3 up_vector(0.0, 0.0, 1.0);

        //cross(外積) dot(内積) normalize(正規化)
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
        if(cuv < threshold)
        {
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
    //outlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(obstacle_passthrough);
    sor.setRadiusSearch(0.05);
    sor.setMinNeighborsInRadius(2);
    sor.filter(*sor_passthrough_cloud); 

    pcl::toROSMsg (*z_passthrough_cloud2, cloud_filtered);
    pcl::toROSMsg (*sor_passthrough_cloud, obstacle2);
    sensor_msgs::convertPointCloud2ToPointCloud(obstacle2, obstacle);

    // Publish the data
    pub.publish (cloud_filtered);
    pub2.publish (obstacle);
    pub3.publish (normals);

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
