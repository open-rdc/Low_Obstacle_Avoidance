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

ros::Publisher pub;

class PoseDrawer
{
    public:
        PoseDrawer() : tf_(), target_frame_("/velodyne_link")
    {
        scan_sub_.subscribe(n_, "/scan2", 10);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(scan_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback(boost::bind(&PoseDrawer::scanCallback, this, _1));
    };

    private:
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
        tf::TransformListener tf_;
        tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
        ros::NodeHandle n_;
        std::string target_frame_;
        laser_geometry::LaserProjection projector_;

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            if(!tf_.waitForTransform(
                        scan_in->header.frame_id,
                        "/base_link",
                        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                        ros::Duration(1.0))){
                return;
            }

            sensor_msgs::PointCloud cloud;
            projector_.transformLaserScanToPointCloud("/base_link",*scan_in,cloud,tf_);
            //projector_.projectLaser(*scan_in, cloud);

            pub.publish (cloud);
        };
};
int main (int argc, char** argv)
{
    ros::init (argc, argv, "laser_to_pointcloud");
    PoseDrawer pd;
    ros::NodeHandle n_;
    //ros::Subscriber sub = nh.subscribe ("/scan2", 40, scanCallback);
    pub = n_.advertise<sensor_msgs::PointCloud> ("/to_pointcloud", 10, 1);
    ros::spin();
};
