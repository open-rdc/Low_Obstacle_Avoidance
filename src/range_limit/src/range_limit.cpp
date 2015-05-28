#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPoinrCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


ros::publisher pub;

void limitCallback (const pcl::PointCloudPtr &cloud)
{
    sensor_msgs::pointCloud2 cloud3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 5;
    cloud->heigt = 1;
    cloud->point.resize (cloud->width * cloud->heigh);

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

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " " 
            << cloud_filtered->points[i].y << " " 
            << cloud_filtered->points[i].z << std::endl;

    pcl::toROSMsg(*cloud_filtered, cloud3);

    pub.publish (cloud3);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "range_limit");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Subscriber sub = nh.shubscribe ("/scan2", 10, limitCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/limit_cloud");

    ros::spin();
}
