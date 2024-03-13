
#include "nmpc_leader.h"
// namespace Leader{
static constexpr double PI = 3.14;


double Leader::degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * PI / 180.0;
}
double Leader::radians2degrees(double degrees_in_angle)
{
    return degrees_in_angle * 180.0 / PI;
}
/*
 * get the euclidian distance between two points
 */
double Leader::getDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

/*
 * Call back implementation to read and process laser data
 */
void Leader::laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{

    int sizeLaser = laser_msg->ranges.size();
    std::vector<double> range(laser_msg->ranges.begin(), laser_msg->ranges.end());
    // sort by distance to check from closer to further away points if they might be something real

    std::sort(range.begin(), range.end());
    int minDistID;
    double minDist = 100; // get max
    if (!lastScan.empty())
    {
        for (int i = 0; i < range.size(); i++)
        {
            double tempMinDist = range[i];
            // #现在，我们检查这是否是噪音：#获取一个窗口。在其中，我们将检查是否有类似距离的扫描#
            // 在该窗口内的begin ,last一次扫描中#我们剪裁窗口，以便索引不会超出边界
            int l = std::max(i - winSize, 0), r = std::min(i + winSize, sizeLaser);
            std::vector<double> window(lastScan.begin() + l, lastScan.begin() + r);
            for (int j = 0; j < window.size(); j++)
            {
                // check if any of the scans in the window (in the last scan) has a distance close enough to the current one
                if (std::abs(window[j] - tempMinDist) <= deltaDist)
                {
                    minDistID = i;
                    minDist = range[i];
                    // 至少有一点同样接近
                    // 这样我们就找到了一个有效的最小值，可以停止循环
                    break;
                }
            }
        }
    }

    lastScan.assign(range.begin(), range.end());
    if (minDist > laser_msg->range_max)
    {
        ROS_INFO(" no obstacle laser ");
    }
    else
    {
        double minDistAngle = laser_msg->angle_min + minDistID * laser_msg->angle_increment;
        // only x angle
        ROS_INFO("minDistAngle=[%f\n]", minDistAngle);
        turtlebot_formation2::position pos;
        pos.angleX = minDistAngle;
        pos.distance = minDist;
        positionPublisher.publish(pos);
    }

    // //ROS_INFO("I am in: [%s]", "laser call back");
    // for (int i=0; i<27; i++) // I need not loop to copy, I am not familiar with std::vectors
    // {
    // intensities [i]= laser_msg->intensities[i];
    // mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
    // }
}
void Leader::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    tf::Quaternion quart;
    tf::quaternionMsgToTF(msg->pose.orientation, quart);
    double roll, pitch, yaw;
    tf::Matrix3x3(quart).getRPY(roll, pitch, yaw); //)
    goal_state_ << x, y, yaw;
    nmpc_ptr_->SetGoalStates(goal_state_);
}

void Leader::current_position_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::cout << "poseCallback" << std::endl;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double w = msg->twist.twist.angular.z;

    tf::Quaternion q;
    double pitch, roll, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    cur_state_ << x, y, yaw, vx, vy, w;
    start_state_ << x, y, yaw;
}
void Leader::visualize()
{
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(lock_vis_);
        vis_ptr_->vis_PointList<std::string>("obs_list", obs_list,std::string("map"));
        vis_ptr_->vis_path_point<Vec3ds,std::string>(preTraj_,"pre_path");
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    
}
void Leader::run()
{
    ros::Rate loop_rate(frequence_);
    geometry_msgs::Twist vel_msg;
    while (LeaderNode.ok()) // ros::ok()
    {
        nmpc_ptr_->Optimize(start_state_);
        Vec2d ctrlComman;
        std::unique_lock<std::mutex> lock(lock_vis_);
        nmpc_ptr_->ComputeCommand(ctrlComman);
        lock.unlock();
        vel_msg.linear.x = ctrlComman(0);
        vel_msg.angular.z = ctrlComman(1);
        velocity_publisher.publish(vel_msg);
        nmpc_ptr_->GetPreTraj(preTraj_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "nmpc_leader");
    ros::NodeHandle nh_leader;
    Leader leader(nh_leader);
    leader.run();
    return 0;
}

// }