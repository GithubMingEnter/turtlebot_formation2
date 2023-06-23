#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <turtlebot_formation2/avoid.h>
double intensities [27];
double mul = 1; 

std::string cmd_vel;
std::string scan;
// tf变换相关参数
std::string base_frame;
std::string base_to_row;
std::string base_to_slave;
std::string tf_prefix_;

geometry_msgs::Twist cmd_vel_avoid;    //avoid velocity
float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向


ros::Publisher slave_vel;
static constexpr double PI=3.14;
class Slaver
{
    //Function declerations to avoid, rotate and move
    void avoid(void);
    void rotate (double angular_speed, double relative_angle, bool clockwise);
    void move(double speed, double distance, bool isForward);

    // Laser call back function decleration
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg); 

    double  degrees2radians(double angle_in_degrees){
        return angle_in_degrees *PI /180.0;
    }
    double  radians2degrees(double degrees_in_angle){
        return degrees_in_angle * 180.0/PI;
    }
    /*
    * get the euclidian distance between two points 
    */
    double  getDistance(double x1, double y1, double x2, double y2){
        return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
    }
};
void Slaver::avoid()
{
    const int samples=27;
    const double inc=degrees2radians(270.0/27.0);
    const int center=samples/2;
    const int range=1%center;
    if((intensities[center-range]==1) || (intensities[center+range]==1) ||(intensities[center]==1)){
        	//Check one by one on both sides of the robot to determine free space and 
            //rotate by the amount scanned in a first free direction
            for(int i=range; i<center; i++){
                if(intensities[center-i]==0)//no obstacle
                {
                    rotate(1.0, (i+1)*inc,1);
                    break;
                }
                else if(intensities[center+i]==0)
                {
                    rotate(1.0,(i+1)*inc,0);//clockwise
                    break;
                }
            }
    }
    else
    {
        
    }
}
/**
 *  makes the robot turn with a certain angular velocity, 
 *  either clockwise or counter-clockwise direction  
 */
void Slaver::rotate (double angular_speed, double relative_angle, bool clockwise){
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
	   //Control strategy for clockwise and counter-clockwise directions
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition to teminate rotation after rotating to the required orientation 
	   do{
		   slave_vel.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   slave_vel.publish(vel_msg);
}


/**
 *  makes the robot turn with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void Slaver::move(double speed, double distance, bool isForward){
   geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis and condition to determine direction of movement
   if (isForward)
	   vel_msg.linear.x =abs(speed);
   else
	   vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   //Condition to teminate movement after raeching the required distance 
   do{
	   slave_vel.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   slave_vel.publish(vel_msg);

}

/*
 * Call back implementation to read and process laser data  
 */
void Slaver::laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{
//ROS_INFO("I am in: [%s]", "laser call back");

  for (int i=0; i<27; i++) // I need not loop to copy, I not familiar with std::vectors
  {
  intensities [i]= laser_msg->intensities[i];
  mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
  }
}

int main(char argc, char** argv)
{


    return 0;
}





