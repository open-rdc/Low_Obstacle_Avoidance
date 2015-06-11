#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <vector>


laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;
ros::Publisher pub;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!listener_.waitForTransform(
                scan_in->header.frame_id,
                "/base_link",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link",*scan_in,cloud,listener_);

    pub.publish (cloud);

    // Do something with cloud.
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "laser_to_pointcloud");
    ros::NodeHandle nh;

    ros::Rate loop_rate(40);

    ros::Subscriber sub = nh.subscribe ("/scan2", 40, scanCallback);

    pub = nh.advertise<sensor_msgs::PointCloud> ("/to_pointcloud", 40, 1);

    ros::spin();
}
