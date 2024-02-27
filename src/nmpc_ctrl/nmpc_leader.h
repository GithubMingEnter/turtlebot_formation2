#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include <nav_msgs/Path.h> 
#include <visualization_msgs/Marker.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <turtlebot_formation2/avoid.h>
#include <turtlebot_formation2/position.h>
#include "nmpc_ctrl.h"
#include "EigenType.h"
double intensities[27];
double mul =1;
double mode = 2;


std::string cmd_vel;
std::string scan;
int winSize=2;
double deltaDist=0.2;
ros::Publisher velocity_publisher;
ros::Publisher positionPublisher;

class Leader{
    ros::Subscriber laser ;
    ros::NodeHandle LeaderNode;
    ros::Subscriber current_position_sub; 
    ros::Subscriber goalSub_; 
    ros::Publisher PrePath_pub_;
    ros::NodeHandle node;
    
    Vec3d goal_state_,start_state_;
    Vec6d cur_state_;

    std:: vector<double> lastScan;
    float angleX;    //障碍物方向
    double frequence_=10;
    std::shared_ptr<NmpcPosCtrl> nmpc_ptr;
    
public:
    float distanceO;    //障碍物距离

    //Function declerations of move and rotate
    void move(double speed, double distance, bool isForward);
    void rotate (double angular_speed, double relative_angle, bool clockwise);

    //Call back decleration for the laser messages
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg);
    void current_position_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    int distance_judgment(void);
    //Function declearations for equilidian distance and degrees to radians conversion
    double getDistance(double x1, double y1, double x2, double y2);
    double degrees2radians(double angle_in_degrees);
    double radians2degrees(double degrees_in_angle);
    
    //Wanader without bumping into obstacles 
    void wander(void);
    void run();
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    Leader(ros::NodeHandle& node):LeaderNode(node)
    {
        // this->laser = LeaderNode.subscribe<sensor_msgs::LaserScan>(scan, 1, &Leader::laserCallBack,this);//?
        
        ros::NodeHandle param("~");
        param.param<std::string>("cmd_vel",cmd_vel,"/tb3_0/cmd_vel");
        param.param<std::string>("scan",scan,"/tb3_0/scan");
        param.param<double>("mode", mode,2);  
        
            //   param.param<double>("mode",mode,"1");
        velocity_publisher = LeaderNode.advertise<geometry_msgs::Twist>(cmd_vel, 1000);
        PrePath_pub_=LeaderNode.advertise<nav_msgs::Path>("/tb3_0/pre_path",100);
        // laser = LeaderNode.subscribe<sensor_msgs::LaserScan>(scan, 1, &Leader::laserCallBack,this);//?
        /***创建障碍物方位话题订阅者***/
    // "/tb3_0/object_tracker/current_position"
    // "/tb3_0/Obstacle_position"
        current_position_sub = LeaderNode.subscribe("/tb3_0/base_pose_ground_truth", 3, &Leader::current_position_Callback,this);
        goalSub_= LeaderNode.subscribe("/move_base_simple/goal",100,&Leader::goal_callback,this);
                                                                                                
        // why can't point msg type <turtlebot_formation2::avoid>
        // positionPublisher=LeaderNode.advertise<turtlebot_formation2::position>("/tb3_0/Obstacle_position",3);
        int predict_step = 60;
        float sample_time=1/frequence_;
        mpc_param mMpcParam;
        mMpcParam.Qvec=std::vector<double>{3.6,6.9,0.8};//Y:1.6 1.5
        mMpcParam.Rvec=std::vector<double>{0.5,0.09};
        mMpcParam.Svec=std::vector<double>{1,3};
        nmpc_ptr=std::make_shared<NmpcPosCtrl>(predict_step, sample_time,mMpcParam);
        nmpc_ptr-> SetSolver();
        nmpc_ptr->SetInput(0.3,-0.3,M_PI/4,-M_PI/4);
    }
    ~Leader(){};

};