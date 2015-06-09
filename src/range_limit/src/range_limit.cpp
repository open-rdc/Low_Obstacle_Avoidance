#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal/h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


ros::Publisher pub;

void limitCallback (const sensor_msgs::PointCloudConstPtr& cloud1)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_filterd (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::convertPointCloudToPointCloud2 (*cloud1, cloud2);
    pcl::fromROSMsg (cloud2, *cloud);
/*    
    cloud->width = 105;
    cloud->height = 25;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i > cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " " 
            << cloud->points[i].y << " " 
            << cloud->points[i].z << std::endl;
*/
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.0, -0.3);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

  /*  std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " " 
            << cloud_filtered->points[i].y << " " 
            << cloud_filtered->points[i].z << std::endl;
*/

    //build the filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSerch(0.8);
    outrem.setMinNeughborsInRadius (2);
    outrem.filter (*cloud_filterd_filterd);


    pcl::toROSMsg(*cloud_filtered_filterd, cloud3);
    pub.publish (cloud3);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "range_limit");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Subscriber sub = nh.subscribe ("/scan2", 10, limitCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/limited_cloud", 10, 1);

    ros::spin();
}
