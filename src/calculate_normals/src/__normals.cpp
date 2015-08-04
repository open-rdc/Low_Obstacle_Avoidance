#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <angles/angles.h>


class normals{
public:
    normals(){
    cloud_sub = nh.subscribe ("/assembled_cloud", 2, &normals::normalCallback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxel_filter_filtered_pcl", 2, 1);
    poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/normal_vectors", 2, 1);
    ros::Rate loop_Rate(40);
}

void normalCallback (const sensor_msgs::PointCloudConstPtr& cloud)
{
}

void calc(){
    sensor_msgs::PointCloud2 cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::convertPointCloudToPointCloud2 (*cloud, cloud1);
    sensor_msgs::PointCloud2 output_normals;
    sensor_msgs::PointCloud2 cloud_normals;
    sensor_msgs::PointCloud2 cloud_filtered;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);   

    pcl::fromROSMsg (cloud1, *cloud2);

/*
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (0.04, 0.04, 0.04);
    sor.filter (*cloud2);
*/
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = cloud2->header.frame_id; //EDITED
    ROS_INFO_STREAM("poseArray.header: frame=" << poseArray.header.frame_id); //Outputs "/map"

    // estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(cloud2);
    ne.setKSearch (24);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*normals);

    for(size_t i = 0; i<normals->points.size(); ++i)
    {
        normals->points[i].x = cloud2->points[i].x;
        normals->points[i].y = cloud2->points[i].y;
        normals->points[i].z = cloud2->points[i].z;

        // Declare goal output pose
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Quaternion msg;

        tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
                  
        //cross(外積)　dot(内積) normalize(正規化)
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        q.normalize();
        double rad = q.getAngle();

        if(!(1.48 < rad && rad < 1.65))
        {
        tf::quaternionTFToMsg(q, msg);

        pose.pose.position.x = normals->points[i].x;
        pose.pose.position.y = normals->points[i].y;
        pose.pose.position.z = normals->points[i].z;

        pose.pose.orientation = msg;

        poseArray.poses.push_back(pose.pose);
        }

    }

    poseArrayPub.publish(poseArray);

    pcl::toROSMsg (*cloud2, cloud_filtered);
    // Publish the data
    pub.publish (cloud_filtered);
    ROS_INFO("poseArray size: %i\n", poseArray.poses.size());
}

void run (){
    while(ros::ok()){
        calc();
        ros::spinOnce();
    }
}

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher pub;
    ros::Publisher poseArrayPub;
    geometry_msgs::PoseArray poseArray;
};

int main (int argc, char** argv)
{
    ros::init (argc, argv, "normal_filter");
    normals normals;
    normals.run();

    return 0;
}
