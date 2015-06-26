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
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Pose.h>

class My_Filter
{
    public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    private:
        tf::TransformListener tfListener_;
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan2", 40, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/to_cloud", 40, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{  
    if(!tfListener_.waitForTransform(
            scan_in->header.frame_id,
            "map",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0))){
        return;
    }
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("map", *scan_in, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "my_filter");
    My_Filter filter;
    ros::spin();
}
