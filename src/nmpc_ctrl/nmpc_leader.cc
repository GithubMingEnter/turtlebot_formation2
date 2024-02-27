
#include "nmpc_leader.h"
// namespace Leader{
static constexpr double PI=3.14;




/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米
入口参数：无
返回  值：1或0
**************************************************************************/
int Leader::distance_judgment(void)
{
	//int a;
	if(distanceO<=0.4) 
	{
		// ROS_INFO("distanceO less then 0.4 ");
		printf("distanceO= %f\n",distanceO);

		return 1;
	}
	else
		return 0;
	
}



/**
 *  makes the robot move with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void  Leader::move(double speed, double distance, bool isForward){
   geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis and check condition for the direction
   if (isForward)
	   vel_msg.linear.x =fabs(speed);
   else
	   vel_msg.linear.x =-fabs(speed);

   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);//adjust
   //Condition to terminate if moved to the distance specified
   do{
	   velocity_publisher.publish(vel_msg);
       double t1=ros::Time::now().toSec();
       current_distance=(t1-t0)*speed;//?
       ros::spinOnce();
       loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   //velocity_publisher.publish(vel_msg);

}

/**
 *  makes the robot turn with a certain angular velocity, for 
 *  a certain distance in either clockwise or counter-clockwise direction  
 */
void Leader::rotate (double angular_speed, double relative_angle, bool clockwise){
//angular_speed = degrees2radians(angular_speed);
//relative_angle = degrees2radians(relative_angle);
	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   //Condition to rotate clockwise or counter-clockwise
           if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition used to terminate if rotated to the specifed angle
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}


/*
 *"Wander without hitting anything" implementation
 */
void Leader::wander(void)
{
    //ROS_INFO("I am [%s]", "wandering");
    int samples = 27;
    int fov = 4.7123;
    double inc = 0.1745; // 270/27 degrees to radians
    int center = samples/2;
    if (mul == 1)// blocked around 270 degrees
    {
    //rotate(1.0, 3.1415, 1); //about turn
    }
    if ((intensities [center-1] == 1)||(intensities [center] == 1)||(intensities [center+1] == 1))// obstacle in front
    {
        //Check one by one on both sides of the robot to determine free space and rotate by the amount scanned in a first free direction
        for (int i = 2; i< center; i++)
        {
            if(intensities [center - i] == 0)// no obstacle
            {
            //rotate(1.0, (i+1)*inc, 1);//move by navigation
            break;
            }
            else if (intensities [center +i] == 0)// no obstacle
            {
            //rotate(1.0, (i+1)*inc, 0);//move by navigation
            break;
            }
        }
    }
    else
    {
        if(mode==1)
            move(0.5, 1.0, 1);  // or move by navigation
        else if(mode==2)
        {
            // ROS_INFO("waitting for navigation");
        }
        else if(mode==3)
        {

            // ROS_INFO(" waitting for teleop");
        }
    }

}


double Leader::degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}
double Leader::radians2degrees(double degrees_in_angle){
	return degrees_in_angle * 180.0/PI;
}
/*
 * get the euclidian distance between two points 
 */
double Leader::getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/*
 * Call back implementation to read and process laser data  
 */
void Leader::laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{

    int sizeLaser=laser_msg->ranges.size();
    std::vector<double> range(laser_msg->ranges.begin(),laser_msg->ranges.end());
    // sort by distance to check from closer to further away points if they might be something real

    std::sort(range.begin(),range.end());
    int minDistID;
    double minDist=100;//get max
    if(!lastScan.empty())
    {
        for(int i=0;i<range.size();i++)
        {
            double tempMinDist=range[i];
            // #现在，我们检查这是否是噪音：#获取一个窗口。在其中，我们将检查是否有类似距离的扫描#
            // 在该窗口内的begin ,last一次扫描中#我们剪裁窗口，以便索引不会超出边界
            int l=std::max(i-winSize,0),r=std::min(i+winSize,sizeLaser);
            std::vector<double> window(lastScan.begin()+l,lastScan.begin()+r);
            for(int j=0;j<window.size();j++)
            {
                // check if any of the scans in the window (in the last scan) has a distance close enough to the current one
                if(std::abs(window[j]-tempMinDist)<=deltaDist)
                {
                    minDistID=i;
                    minDist=range[i];
                    // 至少有一点同样接近
                    //这样我们就找到了一个有效的最小值，可以停止循环
                    break;
                }
            }
            

        }
    }
    
    lastScan.assign(range.begin(),range.end());
    if(minDist>laser_msg->range_max){
        ROS_INFO(" no obstacle laser ");

    }
    else{
        double minDistAngle=laser_msg->angle_min+minDistID*laser_msg->angle_increment;
        //only x angle
        ROS_INFO("minDistAngle=[%f\n]",minDistAngle);
        turtlebot_formation2::position pos;
        pos.angleX=minDistAngle;
        pos.distance=minDist;
        positionPublisher.publish(pos);
    }

    // //ROS_INFO("I am in: [%s]", "laser call back");
    // for (int i=0; i<27; i++) // I need not loop to copy, I am not familiar with std::vectors
    // {
    // intensities [i]= laser_msg->intensities[i];
    // mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
    // }
}
void Leader::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x=msg->pose.position.x;
    double y=msg->pose.position.y;
    tf::Quaternion quart;
    tf::quaternionMsgToTF(msg->pose.orientation,quart);
    double roll,pitch,yaw;
    tf::Matrix3x3(quart).getRPY(roll,pitch,yaw);//)
    goal_state_<<x,y,yaw;
    nmpc_ptr->SetGoalStates(goal_state_);

}

void Leader::current_position_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::cout<<"poseCallback"<<std::endl;
    double x=msg->pose.pose.position.x;
    double y=msg->pose.pose.position.y;
    double vx=msg->twist.twist.linear.x;
    double vy=msg->twist.twist.linear.y;
    double w=msg->twist.twist.angular.z;

    tf::Quaternion q;
    double pitch,roll,yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    cur_state_<<x,y,yaw,vx,vy,w;
    start_state_<<x,y,yaw;
}
void Leader::run(){
    ros::Rate loop_rate(frequence_);
    geometry_msgs::Twist vel_msg;
  while (LeaderNode.ok())//ros::ok()
  {
      nmpc_ptr->Optimize(start_state_);
      Vec2d ctrlComman;
      nmpc_ptr->ComputeCommand(ctrlComman);
      vel_msg.linear.x=ctrlComman(0);
      vel_msg.angular.z=ctrlComman(1);
      velocity_publisher.publish(vel_msg);
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