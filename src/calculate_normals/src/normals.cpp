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
ros::Publisher poseArrayPub;
geometry_msgs::PoseArray poseArray; // particles as PoseArray (preallocated)

void normalCallback (const sensor_msgs::PointCloudConstPtr& in_cloud1)
{
    sensor_msgs::PointCloud2 in_cloud2;
    sensor_msgs::convertPointCloudToPointCloud2 (*in_cloud1, in_cloud2);
    sensor_msgs::PointCloud2 cloud2_filtered;

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_passthrough_cloud (new pcl::PointCloud<pcl::PointXYZ>);   

    float NaN = std::numeric_limits<float>::quiet_NaN();

    pcl::fromROSMsg (in_cloud2, *pcl_cloud);

    poseArray.poses.clear();
    poseArray.header.stamp = ros::Time(0);
    poseArray.header.frame_id = pcl_cloud->header.frame_id; //EDITED
    ROS_INFO_STREAM("poseArray.header: frame=" << poseArray.header.frame_id); //Outputs "/map"
    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (pcl_cloud);
    sor.setLeafSize (0.22, 0.22, 0.22);
    sor.filter (*voxel_cloud);

    //Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud (voxel_cloud);
    z_pass.setFilterFieldName("z");
    z_pass.setFilterLimits (-1.0, 1.0);
    z_pass.filter (*z_passthrough_cloud);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (z_passthrough_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits (0, 10.0);
    pass.filter (*passthrough_cloud);


    // estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(passthrough_cloud);
    ne.setKSearch (24);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*normals);

    /***********************publish normal vectors************************/
    for(size_t i = 0; i<normals->points.size(); ++i)
    {
        normals->points[i].x = passthrough_cloud->points[i].x;
        normals->points[i].y = passthrough_cloud->points[i].y;
        normals->points[i].z = passthrough_cloud->points[i].z;

        // Declare goal output pose
        geometry_msgs::PoseStamped pose;
        geometry_msgs::Quaternion msg;

        tf::Vector3 axis_vector(normals->points[i].normal[0], normals->points[i].normal[1], normals->points[i].normal[2]);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
                  
        //cross($B30@Q(B)$B!!(Bdot($BFb@Q(B) normalize($B@55,2=(B)
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
        if(!((1.76-0.0798) < rad && rad < (1.76+0.0798)))
        //if(!((1.76-0.392) < rad && rad < (1.76+0.392)))
        {
            //ROS_INFO("get_angle: %lf\n", rad);
            passthrough_cloud->points[i].x = NaN;
            passthrough_cloud->points[i].y = NaN;
            passthrough_cloud->points[i].z = NaN;
        }else{
            passthrough_cloud->points[i].x = normals->points[i].x;
            passthrough_cloud->points[i].y = normals->points[i].y;
            passthrough_cloud->points[i].z = normals->points[i].z;
        }
    }

    poseArrayPub.publish(poseArray);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(passthrough_cloud);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(1.0);
    sor2.filter(*sor_passthrough_cloud); 
    
    pcl::toROSMsg (*sor_passthrough_cloud, cloud2_filtered);
 
    // Publish the data
    pub.publish (cloud2_filtered);

    int j;
    j=poseArray.poses.size();
    ROS_INFO("poseArray size: %d\n", j);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "normal_filter");
    ros::NodeHandle nh;

    ros::Rate loop_rate(0.5);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/hokuyo3d/hokuyo_cloud", 1, normalCallback);
    //ros::Subscriber sub = nh.subscribe ("/cloud_pcd", 2, normalCallback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 2000, 1);
    poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/normal_vectors", 1000, 1);

    // Spin
    ros::spin();
}
