#ifndef COMMON_TYPE
#define COMMON_TYPE
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#include <pcl_conversions/pcl_conversions.h>


using Vec2d = Eigen::Vector2d;
using Vec2ds = std::vector<Vec2d>;
using Vec2i = Eigen::Vector2i;

using Vec3d = Eigen::Vector3d;
using Vec3ds = std::vector<Eigen::Vector3d>;

using Vec4d = Eigen::Vector4d;
using Vec4ds = std::vector<Eigen::Vector4d>;

using VecXd=Eigen::VectorXd;

using Matxd=Eigen::MatrixXd;

using PointType=pcl::PointXYZI;



#endif
