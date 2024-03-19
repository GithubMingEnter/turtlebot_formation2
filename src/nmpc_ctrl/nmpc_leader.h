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
#include "vis_ros1.hpp"
#include "fusion_esdf.h"

using namespace vis;
double intensities[27];
double mul =1;
double mode = 2;

std::string pre_path;
std::string cmd_vel;
std::string scan;
std::string odom;
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
    Vec3ds preTraj_;
    Vec3ds obs_list;
    float angleX;    //障碍物方向
    double frequence_=10;
    mpc_param mpcParam_;
    std::shared_ptr<NmpcPosCtrl> nmpc_ptr_;
    std::shared_ptr<displayRviz> vis_ptr_;
    FusionMap::Ptr grid_map_;

    std::mutex lock_vis_;
    std::thread vis_thread_;
    double dist_soft_;
    
public:
    


    //Call back decleration for the laser messages
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg);
    void current_position_Callback(const nav_msgs::Odometry::ConstPtr& msg);

    //Function declearations for equilidian distance and degrees to radians conversion
    double getDistance(double x1, double y1, double x2, double y2);
    double degrees2radians(double angle_in_degrees);
    double radians2degrees(double degrees_in_angle);
    void visualize();

    void run();
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    Leader(ros::NodeHandle& node):LeaderNode(node)
    {
        // this->laser = LeaderNode.subscribe<sensor_msgs::LaserScan>(scan, 1, &Leader::laserCallBack,this);//?
        
        ros::NodeHandle param("~");
        param.param<std::string>("cmd_vel",cmd_vel,"/tb3_0/cmd_vel");
        param.param<std::string>("scan",scan,"/tb3_0/scan");
        param.param<std::string>("odom",odom,"/tb3_0/base_pose_ground_truth");
        param.param<std::string>("pre_path",pre_path,"/tb3_0/pre_path");

        param.param<double>("mode", mode,2);  
        param.param<bool>("isObs",mpcParam_.isObs,true);
        param.param<double>("obsSoftRatio",mpcParam_.obsSoftRatio,10);
        param.param("optimization_ESDF/dist0", dist_soft_, 0.4);
        obs_list=Vec3ds{Vec3d(1,1,0.3),Vec3d(-2,-2,1),Vec3d(3,3,0.3)};
            //   param.param<double>("mode",mode,"1");
        velocity_publisher = LeaderNode.advertise<geometry_msgs::Twist>(cmd_vel, 1000);
        PrePath_pub_=LeaderNode.advertise<nav_msgs::Path>("/tb3_0/pre_path",100);
        // laser = LeaderNode.subscribe<sensor_msgs::LaserScan>(scan, 1, &Leader::laserCallBack,this);//?

        current_position_sub = LeaderNode.subscribe(odom, 3, &Leader::current_position_Callback,this);
        goalSub_= LeaderNode.subscribe("/move_base_simple/goal",100,&Leader::goal_callback,this);
                                                                                                
        // why can't point msg type <turtlebot_formation2::avoid>
        // positionPublisher=LeaderNode.advertise<turtlebot_formation2::position>("/tb3_0/Obstacle_position",3);
        
        vis_ptr_=std::make_shared<displayRviz>(LeaderNode);
        vis_ptr_->enroll<vMarker>("obs_list");
        vis_ptr_->enroll<nav_msgs::Path>("pre_path");

        grid_map_ = std::make_shared<FusionMap>();
        grid_map_->initMap(param);

        int predict_step = 60;
        float sample_time=1/frequence_;


        mpcParam_.Qvec=std::vector<double>{3.6,6.9,0.8};//Y:1.6 1.5
        mpcParam_.Rvec=std::vector<double>{0.5,0.09};
        mpcParam_.Svec=std::vector<double>{1,3};
        nmpc_ptr_=std::make_shared<NmpcPosCtrl>(predict_step, sample_time,mpcParam_);
        nmpc_ptr_->SetObs(obs_list);
        nmpc_ptr_-> SetSolver();
        nmpc_ptr_->SetInput(0.3,-0.3,M_PI/4,-M_PI/4);
        nmpc_ptr_->setFusionMap(grid_map_);
        nmpc_ptr_->setIgnoreDist(dist_soft_);

        vis_thread_=std::thread(&Leader::visualize,this);
    }
    ~Leader(){
        if(vis_thread_.joinable())
            vis_thread_.join();

    };

};