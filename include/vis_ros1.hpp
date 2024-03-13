#ifndef VIS_ROS1
#define VIS_ROS1
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

#include "common_type.h"

namespace vis
{
    using vMarkers = visualization_msgs::MarkerArray;
    using vMarker = visualization_msgs::Marker;

    enum Color
    {
        white,
        red,
        green,
        blue,
        yellow,
        chartreuse,
        black,
        gray,
        orange,
        purple,
        pink,
        steelblue
    };

    class displayRviz
    {

        ros::NodeHandle nh_;
        std::unordered_map<std::string, ros::Publisher> vis_pub_map_;

        void setMarkerPose(visualization_msgs::Marker &marker,
                           const double &x,
                           const double &y,
                           const double &z)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        void setMarkerPose(visualization_msgs::Marker &marker,
                           const double &x,
                           const double &y,
                           const double &z,
                           const geometry_msgs::Quaternion &quat)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation = quat;
        }
        void setMarkerColor(visualization_msgs::Marker &marker,
                            Color color = blue,
                            double a = 1)
        {
            marker.color.a = a>1e-5? a:0.9;
            switch (color)
            {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            }
        }
        void setMarkerScale(visualization_msgs::Marker &marker,
                            const double &x,
                            const double &y,
                            const double &z)
        {
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
        }
        void setMarkerScale(visualization_msgs::Marker &marker,
                            const Vec3d &scale_xyz)
        {
            marker.scale.x = scale_xyz(0);
            marker.scale.y = scale_xyz(1);
            marker.scale.z = scale_xyz(2);
        }
        void setMarkerScale(visualization_msgs::Marker &marker,
                                    const double &scale)
        {
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
        }    
    public:
        ~displayRviz(){};
        displayRviz(ros::NodeHandle &nh) : nh_(nh){

                                           };
        template <class _pub_type, class _topic>
        void enroll(const _topic &topic)
        {
            std::unordered_map<std::string, ros::Publisher>::iterator pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<_pub_type>(topic, 8);
                vis_pub_map_[topic] = vis_pub;
            }
        }
        template <class _pub_type, class _topic>
        void enroll(const _topic &topic, uint32_t queue_size)
        {
            std::unordered_map<std::string, ros::Publisher>::iterator pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<_pub_type>(topic, queue_size);
                vis_pub_map_[topic] = vis_pub;
            }
        }
        //
        template <class _path, class _topic>
        void vis_path(const _path &path, const _topic &topic, const std::string &frame = "map")
        {
            // TODO note whether to enroll
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<nav_msgs::Path>(topic, 8);
                vis_pub_map_[topic] = vis_pub;
            }
            const std::string frame_path = frame;
            nav_msgs::Path tmp_path;
            geometry_msgs::PoseStamped tmp_poses;
            // geometry_msgs::Pose tmp_pose;
            tmp_poses.header.frame_id = frame_path;
            for (const auto &pt : path)
            {
                tmp_poses.pose.position.x = pt[0];
                tmp_poses.pose.position.y = pt[1];
                tmp_poses.pose.position.z = pt[2];
                tmp_path.poses.emplace_back(tmp_poses);
            }
            tmp_path.header.frame_id = frame_path;
            tmp_path.header.stamp = ros::Time::now();
            vis_pub_map_[topic].publish(tmp_path);
        }
        template <class _point, class _topic>
        void vis_pointcloud(const _point &ptc, const _topic &topic, const std::string &frame = "map")
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 8);
                vis_pub_map_[topic] = vis_pub;
            }
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            sensor_msgs::PointCloud2 point_cloud_msg;
            point_cloud.reserve(ptc.size());
            for (const auto &pt : ptc)
            {
                point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
            }
            pcl::toROSMsg(point_cloud, point_cloud_msg);
            point_cloud_msg.header.frame_id = frame;
            point_cloud_msg.header.stamp = ros::Time::now();
            vis_pub_map_[topic].publish(point_cloud_msg);
        }

        template <class _point, class _topic>
        void vis_pointcloud_intensity(const _point &ptc, const _topic &topic, const std::string &frame = "map")
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 8);
                vis_pub_map_[topic] = vis_pub;
            }
            pcl::PointCloud<pcl::PointXYZI> point_cloud;
            sensor_msgs::PointCloud2 point_cloud_msg;
            point_cloud.reserve(ptc.size());
            for (const auto &pt : ptc)
            {
                point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
            }
            pcl::toROSMsg(point_cloud, point_cloud_msg);
            point_cloud_msg.header.frame_id = frame;
            point_cloud_msg.header.stamp = ros::Time::now();
            vis_pub_map_[topic].publish(point_cloud_msg);
        }
        // display point_cloud directly
        template <class _cloud, class _topic>
        void vis_pcl(const _cloud &ptc, const _topic &topic, const std::string &frame = "map")
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 8);
                vis_pub_map_[topic] = vis_pub;
            }
            sensor_msgs::PointCloud2 point_cloud_msg;
            pcl::toROSMsg(ptc, point_cloud_msg);
            point_cloud_msg.header.frame_id = frame;
            point_cloud_msg.header.stamp = ros::Time::now();
            vis_pub_map_[topic].publish(point_cloud_msg);
        }

        template <class _path_point, class _topic>
        void vis_path_point(const _path_point &path_point, const _topic &topic, const std::string &frame = "map")
        {

            vis_path(path_point, topic);
            vis_pointcloud(path_point, std::string(topic) + "cloud");
        }

        template <class _path_point, class _topic>
        void vis_path_strip(const _path_point &path_point, const _topic &topic, const std::string &frame = "map", double vscale = 0.03, const Color color = red)
        {

            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarkers>(topic, 9);
                vis_pub_map_[topic] = vis_pub;
            }

            vMarkers path_strip_msg;
            vMarker path_msg;
            path_strip_msg.markers.reserve(path_point.size() + 1);
            // only display the current path
            // vMarker clear_previous_msg;
            // clear_previous_msg.action = vMarker::DELETEALL;
            // path_strip_msg.markers.emplace_back(clear_previous_msg);
            path_msg.type = vMarker::LINE_STRIP;
            path_msg.action = vMarker::ADD;
            path_msg.header.frame_id = frame;
            path_msg.id = 0;
            setMarkerPose(path_msg, 0, 0, 0);
            setMarkerScale(path_msg, vscale, vscale, vscale);
            setMarkerColor(path_msg, color);

            for (const auto &path : path_point)
            {

                path_msg.points.resize(path.size());
                for (size_t i = 0; i < path.size(); ++i)
                {
                    ROS_INFO_STREAM("X: " << path[i].x() << " Y: " << path[i].y());
                    path_msg.points[i].x = path[i].x();
                    path_msg.points[i].y = path[i].y();
                    path_msg.points[i].z = path[i].z();
                }
                path_strip_msg.markers.emplace_back(path_msg);
                ++path_msg.id;
            }
            vis_pub_map_[topic].publish(path_strip_msg);
        }
        template <class _path_point, class _topic>
        void vis_path_robot(const _path_point &path_point, const _topic &topic, double start_yaw, double length,
                            double width, double high = 1.0, const std::string &frame = "map", const Color color = red)
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarkers>(topic, 9);
                vis_pub_map_[topic] = vis_pub;
            }

            vMarkers path_robot_msg;
            vMarker path_msg;
            path_robot_msg.markers.reserve(path_point.size() + 1);
            // only display the current path
            // vMarker clear_previous_msg;
            // clear_previous_msg.action = vMarker::DELETEALL;
            // path_strip_msg.markers.emplace_back(clear_previous_msg);
            path_msg.type = vMarker::CUBE;
            path_msg.action = vMarker::ADD;
            path_msg.header.frame_id = frame;
            path_msg.id = 0;
            tf2::Vector3 axis(0, 0, 1);
            geometry_msgs::Quaternion q;
            tf2::Quaternion start_q(axis, start_yaw);
            q = tf2::toMsg(start_q);
            setMarkerPose(path_msg, path_point[0].x(), path_point[0].y(), path_point[0].z(), q);
            setMarkerScale(path_msg, length, width, high);
            setMarkerColor(path_msg, color, 0.2);
            path_msg.id = 0;
            path_robot_msg.markers.emplace_back(path_msg);//first 
            auto last_vis=path_point[0];
            for (size_t i = 1; i < path_point.size(); ++i)
            {
                //display every robot length
                if((path_point[i]-last_vis).norm()<=length)
                    continue;

                double dy = path_point[i].y() - path_point[i - 1].y();
                double dx = path_point[i].x() - path_point[i - 1].x();
                double path_yaw = atan2(dy, dx);
                tf2::Quaternion path_q(axis, path_yaw);

                q = tf2::toMsg(path_q);

                setMarkerPose(path_msg, path_point[i].x(), path_point[i].y(), path_point[i].z(), q);
                path_msg.id = i;
                last_vis=path_point[i];
                path_robot_msg.markers.emplace_back(path_msg);
            }

            vis_pub_map_[topic].publish(path_robot_msg);
        }

        template <class _topic>
        void vis_arrow(const geometry_msgs::Point &st, const geometry_msgs::Point end, const _topic &topic,
                       const Vec3d &scale_xyz, const int32_t action_type = vMarker::ADD, const std::string &frame = "map", const Color color = red)
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarker>(topic, 1);
                vis_pub_map_[topic] = vis_pub;
            }
            vMarker arrow;
            arrow.header.frame_id = frame;
            arrow.header.stamp = ros::Time::now();
            arrow.id = 0;
            arrow.action = action_type;
            arrow.ns = "arrow";
            setMarkerScale(arrow, scale_xyz);
            setMarkerColor(arrow, color, 0.9);
            arrow.points.emplace_back(st);
            arrow.points.emplace_back(end);
            vis_pub_map_[topic].publish(arrow);
        }
        template <class _topic>
        void vis_single_marker(const geometry_msgs::Point pt, const _topic &topic,
                               const Vec3d &scale_xyz, int32_t marker_type = vMarker::SPHERE,
                               const std::string &frame = "map", const Color color = red)
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarker>(topic, 1);
                vis_pub_map_[topic] = vis_pub;
            }
            vMarker single_marker;
            single_marker.header.frame_id = frame;
            single_marker.header.stamp = ros::Time::now();
            single_marker.id = 0;
            single_marker.type = marker_type;
            single_marker.action = vMarker::ADD;
            setMarkerPose(single_marker, pt.x, pt.y, pt.z);
            setMarkerColor(single_marker, color);
            setMarkerScale(single_marker, scale_xyz);
            vis_pub_map_[topic].publish(single_marker);
        }
        template <class _topic>
        void vis_poly(Vec3ds pts, const _topic &topic, const std::string &frame = "map")
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {

                ros::Publisher vis_pub = nh_.advertise<geometry_msgs::PolygonStamped>(topic, 3);
                vis_pub_map_[topic] = vis_pub;
            }
            geometry_msgs::PolygonStamped polys;
            polys.header.frame_id = frame;
            polys.header.stamp = ros::Time::now();
            for (int i = 0; i < pts.size(); i++)
            {
                geometry_msgs::Point32 p;
                p.x = pts[i](0);
                p.y = pts[i](1);
                p.z = pts[i](2);
                polys.polygon.points.emplace_back(p);
            }
            vis_pub_map_[topic].publish(polys);
        }
        template <class _topic>
        void vis_PointList(const _topic &topic, const std::vector<Eigen::Vector2d> &list, double scale, 
										const Color color = blue, int id=0)
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarker>(topic, 3);
                vis_pub_map_[topic] = vis_pub;
            }
            vMarker sphere;
            sphere.header.frame_id  = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.type = vMarker::SPHERE_LIST;
            sphere.action = vMarker::ADD;
            sphere.id = id;

            sphere.pose.orientation.w = 1.0;
            setMarkerColor(sphere, color);
            setMarkerScale(sphere, scale);

            geometry_msgs::Point pt;
            for (int i = 0; i < int(list.size()); i++)
            {
                pt.x = list[i](0);
                pt.y = list[i](1);
                pt.z = 0;
                sphere.points.emplace_back(pt);
            }
            vis_pub_map_[topic].publish(sphere);
        }
        template <class _topic>
        void vis_PointList(const _topic &topic, const std::vector<Eigen::Vector3d> &list,const std::string frame_id="map",
										const Color color = blue, int id=0)
        {
            auto pub_iter = vis_pub_map_.find(topic);
            if (pub_iter == vis_pub_map_.end())
            {
                ros::Publisher vis_pub = nh_.advertise<vMarker>(topic, 3);
                vis_pub_map_[topic] = vis_pub;
            }
            vMarker sphere;
            sphere.header.frame_id  = frame_id;
            sphere.header.stamp = ros::Time::now();
            sphere.type = vMarker::SPHERE_LIST;
            sphere.action = vMarker::ADD;
            sphere.id = id;

            sphere.pose.orientation.w = 1.0;
            setMarkerColor(sphere, color);
            setMarkerScale(sphere, 0.3);

            geometry_msgs::Point pt;
            for (int i = 0; i < int(list.size()); i++)
            {
                pt.x = list[i](0);
                pt.y = list[i](1);
                pt.z = 0;
                sphere.points.emplace_back(pt);
            }
            vis_pub_map_[topic].publish(sphere);
        }
    };

}
#endif
