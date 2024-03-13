
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include<std_msgs/String.h>

std::string robot_name;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout<<"poseCallback"<<std::endl;
    static tf::TransformBroadcaster TfBroad;
    tf::Transform tfTrans;
    tfTrans.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, 0.0));//z isn't pointed

    tf::Quaternion q;
    double pitch,roll,yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    q.setRPY(0.0, 0.0 , yaw);//only yaw
    tfTrans.setRotation(q);
    TfBroad.sendTransform(tf::StampedTransform(tfTrans, ros::Time::now(), "map" , robot_name));
    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Formation_tf_broadcaster");
    if(argc!=2){
        ROS_ERROR("need a robotic name");
        return -1;
    };
    robot_name=argv[1];
    std::cout<<robot_name<<std::endl;
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe(robot_name + "/odom", 10 , &poseCallback);
    // ros::Subscriber sub=nh.subscribe(robot_name + "/base_pose_ground_truth", 10 , &poseCallback);

    ros::spin();
    return 0;
}






