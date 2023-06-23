#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <turtlebot_formation2/avoid.h>
#include <turtlebot_formation2/position.h>
using namespace std;
 
geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_avoid;    //速度控制信息数据
geometry_msgs::Twist cmd_vel_data;    //速度控制信息数据
ros::Publisher cmd_vel_Pub;
float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向
/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const turtlebot_formation2::position& msg)	
{
	distance1 = msg.distance;
	dis_angleX = msg.angleX;
}


/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void cmd_vel_ori_Callback(const nav_msgs::Odometry& msg)
{
	//note
	ROS_INFO("ODOM");
	cmd_vel_msg.linear.x = msg.twist.twist.linear.x;
	cmd_vel_msg.angular.z =  msg.twist.twist.linear.x;

	cmd_vel_data.linear.x = msg.twist.twist.linear.x;
	cmd_vel_data.angular.z =  msg.twist.twist.linear.x;
}

/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米
入口参数：无
返回  值：1或0
**************************************************************************/
int distance_judgment(void)
{
	//int a;
	if(distance1<=0.7) 
	{
		ROS_INFO("distance1 less then 0.4 ");
		printf("distance1= %f\n",distance1);

		return 1;
	}
	else
		return 0;
	
}
 
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上
入口参数：无
返回  值：1或0
**************************************************************************/
int dis_angleX_judgment(void)
{
	if(cmd_vel_msg.linear.x > 0 && (dis_angleX >  2.335 || dis_angleX < -2.335))
		{
		ROS_INFO("dis_angleX_judgment");

		ROS_INFO("dis_angleX_judgment");
		int temp_count = 0;    //计数变量

		if(((dis_angleX >  2.75 && dis_angleX < 3.14)) || ((dis_angleX >  -3.14 && dis_angleX < -2.75)) )// 障碍在机器人正前方
			{
			temp_count++; 
			if(temp_count > 5)
			{
				cmd_vel_avoid.linear.x  =  -0.3;
				cmd_vel_avoid.linear.y  =  0;
				cmd_vel_avoid.angular.z  =  0.3;
				temp_count = 0;
			}
			else{
				cmd_vel_avoid.linear.x  =  -0.25; // 后退转左
				cmd_vel_avoid.linear.y  =  0.0;
				cmd_vel_avoid.angular.z = 0.3;

			}

			ROS_INFO("turn left ");

			}
		else if(dis_angleX >  2 && dis_angleX < 2.75) // 障碍在机器人右前方
			{
			cmd_vel_avoid.linear.x  =  0.12; // 转左
			cmd_vel_avoid.linear.y  =  0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn left ");
			}
		else if(dis_angleX >  -2.75 && dis_angleX < -2) // 障碍在机器人左前方
			{
			cmd_vel_avoid.linear.x  = 0.12;   // 转右
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = -0.25;
			ROS_INFO("turn right ");
			}
		else
			{
			cmd_vel_avoid.linear.x  = 0.12;  // 其他情况
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn right ");
			}
		return 1;
		}
	else if(cmd_vel_msg.linear.x > 0 && (dis_angleX >  -2 && dis_angleX < -1.67))  // 障碍在机器人左侧
		{
			cmd_vel_avoid.linear.x  = 0.12;  // 转右
			cmd_vel_avoid.linear.y  = 0.0;
			cmd_vel_avoid.angular.z = -0.25;
			ROS_INFO("turn right ");
		return 1;
		}
	else if(cmd_vel_msg.linear.x > 0  && (dis_angleX > 1.67 && dis_angleX < 2)) // 障碍在机器人右侧
		{
			cmd_vel_avoid.linear.x  =  0.12;   // 转左
			cmd_vel_avoid.linear.y  =  0.0;
			cmd_vel_avoid.angular.z = 0.25;
			ROS_INFO("turn left ");
		return 1;
		}
		
	else 
		return 0;
		
}

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


geometry_msgs::Twist  rotate_vel_msg;
geometry_msgs::Twist  move_vel_msg;



ros::Publisher slave_vel;
static constexpr double PI=3.14;
class Slaver
{
    ros::NodeHandle node;
    ros::Subscriber current_position_sub; 
    
    
public:
    float distanceO;    //障碍物距离
    float angleX;    //障碍物方向
    int temp_count=0;
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

			temp_count++;
			if(temp_count > 5)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点
			{
				ROS_INFO("Obstacle detected, implement avoidance action.");
   				if (fabs(cmd_vel_avoid.linear.x)>0.05) // 排除噪点
   			 	{
					cmd_vel_msg = cmd_vel_avoid;
   				 }
  			 	else 
 		   		{
					cmd_vel_avoid.linear.x  =  0;  
					cmd_vel_avoid.linear.y  =  0;
					cmd_vel_avoid.angular.z =  0;
   			 	}
				cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人
			}
		
		else
		{
			// ROS_INFO("Obstacle clear.");
			// if (fabs(cmd_vel_data.linear.x)>0.05)   //排除噪点
   			// {
			// 	cmd_vel_msg = cmd_vel_data;
   			// }
  			// else 
 		    // {
			// 	cmd_vel_data.linear.x  =  0;  
			// 	cmd_vel_data.linear.y  =  0;
			// 	cmd_vel_data.angular.z =  0;
   			// }
			// temp_count = 0;    //排除雷达噪点
			// cmd_vel_msg = cmd_vel_data;
			// cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人
		}

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
	/***创建底盘速度控制话题发布者***/
	 cmd_vel_Pub = node.advertise<geometry_msgs::Twist>(cmd_vel, 1);

	/***创建底盘运动话题订阅者***/
	ros::Subscriber vel_sub = node.subscribe(odom, 1, cmd_vel_ori_Callback);

  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe(base_to_row+"/object_tracker/current_position", 1, current_position_Callback);


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
        if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向
		{
                slaver.avoid();
        }
        else{
                geometry_msgs::Twist vel_msg;
            vel_msg.angular.z=4.0*atan2(slaver_tf.transformSM.getOrigin().y(),
                                                                            slaver_tf.transformSM.getOrigin().x());
            vel_msg.linear.x= 0.5*sqrt(pow(slaver_tf.transformSM.getOrigin().x(),2)+
                                                                    pow(slaver_tf.transformSM.getOrigin().y(),2));
                slave_vel.publish(vel_msg);          
        }
        // slaver_tf.logTf();
       //controller to follow the formation

        ros::spinOnce();
        rate.sleep();                                               

    }
    return 0;
}





