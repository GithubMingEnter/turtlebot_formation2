#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void speedCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 处理接收到的速度消息
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // 打印接收到的速度数据
    ROS_INFO("Received linear speed: %f, angular speed: %f", linear_x, angular_z);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "speed_subscriber");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个订阅器，订阅名为"/cmd_vel"的速度话题
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, speedCallback);

    // 循环等待ROS消息
    ros::spin();

    return 0;
}
