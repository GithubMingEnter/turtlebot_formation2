#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <turtlebot_formation2/avoid.h>
#include <turtlebot_formation2/position.h>
double intensities [27];
double mul = 1; 

std::string cmd_vel;
std::string odom;
std::string scan;
// tf变换相关参数
std::string base_frame;
std::string base_to_row;
std::string base_to_slave;
std::string tf_prefix_;

geometry_msgs::Twist cmd_vel_avoid;    //avoid velocity
geometry_msgs::Twist  rotate_vel_msg;
geometry_msgs::Twist  move_vel_msg;
float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向


ros::Publisher slave_vel;
static constexpr double PI=3.14;
class Slaver
{
    ros::NodeHandle node;
    ros::Subscriber current_position_sub; 
    
    
public:
    float distanceO;    //障碍物距离
    float angleX;    //障碍物方向
    Slaver(ros::NodeHandle & node):node(node)

    {
        
          slave_vel =node.advertise<geometry_msgs::Twist>(cmd_vel, 10);
            ros::Subscriber laser = node.subscribe<sensor_msgs::LaserScan>(scan, 1, &Slaver::laserCallBack,this);
                //  current_position_sub = node.subscribe("/tb3_1/object_tracker/current_position", 
                //                                                                                         3, &Slaver::current_position_Callback,this);
    }
    //Function declerations to avoid, rotate and move
    void avoid(void);
    void rotate (double angular_speed, double relative_angle, bool clockwise);
    void move(double speed, double distance, bool isForward);
    int distance_judgment(void);
    // Laser call back function decleration
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg); 
    void current_position_Callback(const turtlebot_formation2::position& msg);
    inline double  degrees2radians(double angle_in_degrees){
    return angle_in_degrees *PI /180.0;
    }
    inline double  radians2degrees(double degrees_in_angle){
        return degrees_in_angle * 180.0/PI;
    }
    /*
    * get the euclidian distance between two points 
    */
    inline double  getDistance(double x1, double y1, double x2, double y2){
        return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
    }

};
void Slaver::current_position_Callback(const turtlebot_formation2::position& msg)
{
    distanceO=msg.distance;
    ROS_INFO("distance: [%f\n]",distanceO);

    angleX=msg.angleX;
}
int Slaver::distance_judgment(void)
{
	//int a;
	if(distanceO<=0.3) 
	{
		ROS_INFO("distanceO less then 0.3 ");
		printf("distanceO= %f\n",distanceO);

		return 1;
	}
	else
		return 0;
}
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上
入口参数：无
返回  值：int
**************************************************************************/
// int dis_angleX_judge()
// {
//     if(velo)


// }

void Slaver::avoid()
{

    // const int samples=27;
    // const double inc=degrees2radians(270.0/27.0);
    // const int center=samples/2;
    // const int range=1%center;
    // if((intensities[center-range]==1) || (intensities[center+range]==1) ||(intensities[center]==1)){
    //     	//Check one by one on both sides of the robot to determine free space and 
    //         //rotate by the amount scanned in a first free direction
    //         for(int i=range; i<center; i++){
    //             if(intensities[center-i]==0)//no obstacle
    //             {
    //                 rotate(1.0, (i+1)*inc,1);
    //                 break;
    //             }
    //             else if(intensities[center+i]==0)
    //             {
    //                 rotate(1.0,(i+1)*inc,0);//clockwise
    //                 break;
    //             }
    //         }
    // }
    // else
    // {
        
    // }
}

/**
 *  makes the robot turn with a certain angular velocity, 
 *  either clockwise or counter-clockwise direction  
 */
void Slaver::rotate (double angular_speed, double relative_angle, bool clockwise){
//angular_speed = degrees2radians(angular_speed);
//relative_angle = degrees2radians(relative_angle);
	// geometry_msgs::Twist  rotate_vel_msg;
	   //set a random linear velocity in the x-axis
	    rotate_vel_msg.linear.x =0;
	    rotate_vel_msg.linear.y =0;
	    rotate_vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	    rotate_vel_msg.angular.x = 0;
	    rotate_vel_msg.angular.y = 0;
	   //Control strategy for clockwise and counter-clockwise directions
	   if (clockwise)
	   		    rotate_vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		    rotate_vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition to teminate rotation after rotating to the required orientation 
	   do{
		   slave_vel.publish( rotate_vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   
	   }while(current_angle<relative_angle);
	    rotate_vel_msg.angular.z =0;
	   slave_vel.publish( rotate_vel_msg);
}


/**
 *  makes the robot turn with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void Slaver::move(double speed, double distance, bool isForward){
   geometry_msgs::Twist  move_vel_msg;
   //set a random linear velocity in the x-axis and condition to determine direction of movement
   if (isForward)
	    move_vel_msg.linear.x =abs(speed);
   else
	    move_vel_msg.linear.x =-abs(speed);
    move_vel_msg.linear.y =0;
    move_vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
    move_vel_msg.angular.x = 0;
    move_vel_msg.angular.y = 0;
    move_vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(10.0);
   //Condition to teminate movement after raeching the required distance 
   do{
	   slave_vel.publish( move_vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
    move_vel_msg.linear.x =0;
   slave_vel.publish( move_vel_msg);

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
class SlaverTf{
public:
    tf::StampedTransform transformSM;
    tf::StampedTransform transformMS;
    tf::TransformListener listener;

    void slaverToLeader()
    {
   
    }
    void logTf()
    {
        //Priting the x, y, and angle "from the robot's perspective" of each robot to the other
        ROS_INFO("Master pose w.r.t Slave [x, y]: [%f, %f]", transformSM.getOrigin().x(), transformSM.getOrigin().y());
        ROS_INFO("Orientation: [%f]", atan2(transformSM.getOrigin().y(),
                                    transformSM.getOrigin().x()));
        ROS_INFO("---------------------------------------------");
        ROS_INFO("Slave pose w.r.t Master [x, y]: [%f, %f]", transformMS.getOrigin().x(), transformMS.getOrigin().y());
        ROS_INFO("Orientation: [%f]", atan2(transformMS.getOrigin().y(),
                                    transformMS.getOrigin().x()));       
    }

};
int main(int argc, char** argv)
{
        ros::init(argc, argv, "slaver_tf");
        
       ros::NodeHandle paramLoad("~");
        paramLoad.param<std::string>("cmd_vel" ,cmd_vel, "/tb3_1/cmd_vel");
        paramLoad.param<std::string>("odom" ,odom, "/tb3_1/odom");
        paramLoad.param<std::string>("scan", scan,"/tb3_1/scan");
        paramLoad.param<std::string>("base_frame", base_frame, "/tb3_0");
        paramLoad.param<std::string>("base_to_row", base_to_row, "/tb3_1");
        paramLoad.param<std::string>("base_to_slave", base_to_slave, "slave2");
    tf_prefix_ = tf::getPrefixParam(paramLoad); 
    std::cout<<"tf_prefix_="<<tf_prefix_<<std::endl;

    ros::NodeHandle node; 
    Slaver slaver(node);
    SlaverTf slaver_tf;

      ros::Rate rate(10.0);

    while(node.ok()){//ros::ok()
        //from slave to leader  tf transformation
        try{
            slaver_tf.listener.lookupTransform(base_to_row,base_frame,
                                    ros::Time(0), slaver_tf.transformSM);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //from leader to slaver
        try{
            slaver_tf.listener.lookupTransform(base_frame,base_to_row,
                                    ros::Time(0), slaver_tf.transformMS);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        slaver.avoid();
        // slaver_tf.logTf();
       //controller to follow the formation
       geometry_msgs::Twist vel_msg;
       vel_msg.angular.z=4.0*atan2(slaver_tf.transformSM.getOrigin().y(),
                                                                     slaver_tf.transformSM.getOrigin().x());
       vel_msg.linear.x= 0.5*sqrt(pow(slaver_tf.transformSM.getOrigin().x(),2)+
                                                             pow(slaver_tf.transformSM.getOrigin().y(),2));
        slave_vel.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();                                               

    }
    return 0;
}





