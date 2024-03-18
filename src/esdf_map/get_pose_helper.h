/*
 * snap_utils
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

namespace pose_utils
{

class GetPoseHelper
{
public:
  GetPoseHelper(tf2_ros::Buffer* tf,
    const std::string& base_frame,
    const std::string& odom_frame)
  : tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
  {
  };

  bool getOdomPose(Eigen::Matrix4f& pose, const ros::Time& t)
  {
    geometry_msgs::TransformStamped base_ident, odom_pose;
    base_ident.header.stamp = t;
    base_ident.header.frame_id = base_frame_;
    base_ident.transform.rotation.w = 1.0;

    try
    {
      odom_pose = tf_->transform(base_ident, odom_frame_);
    }
    catch(tf2::TransformException e)
    {
      // ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }
    
    // const double yaw = tf2::getYaw(odom_pose.transform.rotation);
    // transfer geometry_msgs::TransformStamped to Eigen::Matrix4f
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) << odom_pose.transform.translation.x,
      odom_pose.transform.translation.y,
      odom_pose.transform.translation.z;
    pose.block<3, 3>(0, 0) = Eigen::Quaternionf(odom_pose.transform.rotation.w,
        odom_pose.transform.rotation.x,
        odom_pose.transform.rotation.y,
        odom_pose.transform.rotation.z)
        .toRotationMatrix();

    return true;
  };

private:
  tf2_ros::Buffer* tf_;
  std::string base_frame_, odom_frame_;
};

}