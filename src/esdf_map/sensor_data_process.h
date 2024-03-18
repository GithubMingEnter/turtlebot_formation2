#pragma once

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <daimon_tool/pcl_df_utils.h>
#include <daimon_tool/get_pose_helper.h>

// spdlog
#include <spdlog/spdlog.h>

class SensorDataProcess{
public:
    SensorDataProcess(ros::NodeHandle &public_nh);
    ~SensorDataProcess();

private:
    ros::NodeHandle nh_;

    std::string base_link_frame_;
    std::string odom_frame_;
    std::string scan_frame_;
    std::string cam_frame_;
    std::string map_fram_;
    std::string scan_topic_;
    std::string cam_topic_;
    
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    geometry_msgs::TransformStamped::Ptr TF_base_to_cam_ptr_;
    geometry_msgs::TransformStamped::Ptr TF_base_to_scan_ptr_;

    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tfL_;

    std::unique_ptr<pose_utils::GetPoseHelper> pose_helper_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_filter_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> depth_cam_filter_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> depth_cam_filter_;

    Eigen::Matrix4f base_to_scan_mat_;
    Eigen::Matrix4f base_to_cam_mat_; 

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> fused_cloud_ptr_;

    ros::Timer process_timer_;
    ros::Publisher fusion_point_pub_;
    ros::Time nearst_time_;
    bool update_point_cloud_;

    boost::mutex point_cloud_mutex_;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void depthCamCallback(const sensor_msgs::PointCloud2::ConstPtr& depth_cloud);
    void processTimerCallback(const ros::TimerEvent& event);
};
