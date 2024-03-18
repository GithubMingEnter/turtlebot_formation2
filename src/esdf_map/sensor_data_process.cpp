#include <daimon_fusion_map/sensor_data_process.h>

SensorDataProcess::SensorDataProcess(ros::NodeHandle &public_nh)
    : nh_(public_nh) {

    nh_.param("local_map/base_id", base_link_frame_, std::string("base_link"));
    nh_.param("local_map/odom_carto", odom_frame_, std::string("odom"));
    nh_.param("local_map/scan_sensor_frame_id", scan_frame_, std::string("base_laser"));
    nh_.param("local_map/pointcloud_sensor_frame_id", cam_frame_, std::string("base_foot_link"));
    nh_.param("map_frame", map_fram_, std::string("map"));
    nh_.param("sensor/scan_topic", scan_topic_, std::string("/scan"));
    nh_.param("sensor/cam_topic", cam_topic_, std::string("/depthCamera_fused/depth/color/points"));

    std::cout<<"sensor/scan_topic "<<scan_topic_<<std::endl;
    std::cout<<"sensor/cam_topic "<<cam_topic_<<std::endl;
    std::cout<<"local_map/scan_sensor_frame_id "<<scan_frame_<<std::endl;
    
    TF_base_to_cam_ptr_.reset(new geometry_msgs::TransformStamped);
    TF_base_to_scan_ptr_.reset(new geometry_msgs::TransformStamped);

    tf2_ros::Buffer tfBuffer_scan, tfBuffer_pc;
    tf2_ros::TransformListener tfListener_scan(tfBuffer_scan), tfListener_pc(tfBuffer_pc);

    try
    {
        *(TF_base_to_scan_ptr_) = tfBuffer_scan.lookupTransform(base_link_frame_, scan_frame_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        spdlog::warn("fail to listen the transform of {}->{}. Error message: {}", base_link_frame_, scan_frame_, ex.what());
        return;
    }

    try
    {
        *(TF_base_to_cam_ptr_) = tfBuffer_pc.lookupTransform(base_link_frame_, cam_frame_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        spdlog::warn("fail to listen the transform of {}->{}. Error message: {}", base_link_frame_, cam_frame_, ex.what());
        return;
    }

    base_to_cam_mat_ = tf2::transformToEigen(*(TF_base_to_cam_ptr_)).matrix().cast<float>();
    base_to_scan_mat_ = tf2::transformToEigen(*(TF_base_to_scan_ptr_)).matrix().cast<float>();

    fused_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    update_point_cloud_ = false;

    fusion_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fused_point_cloud", 10);

    tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(30.0));
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, scan_topic_, 10);
    scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*scan_filter_sub_, *tf_, odom_frame_, 10, nh_);
    scan_filter_->registerCallback(boost::bind(&SensorDataProcess::laserCallback, this, _1));

    depth_cam_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, cam_topic_, 10);
    depth_cam_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>>(*depth_cam_filter_sub_, *tf_, odom_frame_, 10, nh_);
    depth_cam_filter_->registerCallback(boost::bind(&SensorDataProcess::depthCamCallback, this, _1));

    process_timer_ = nh_.createTimer(ros::Duration(0.1), &SensorDataProcess::processTimerCallback, this);
    pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(tf_.get(), base_link_frame_, odom_frame_);
}

SensorDataProcess::~SensorDataProcess() {
    fused_cloud_ptr_.reset();
    pose_helper_.reset();
    TF_base_to_cam_ptr_.reset();
    TF_base_to_scan_ptr_.reset();
}

void SensorDataProcess::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    Eigen::Matrix4f odom_to_base_mat;
    nearst_time_ = scan->header.stamp;
    if (scan->ranges.size() == 0) {
        return;
    }
    if(!pose_helper_->getOdomPose(odom_to_base_mat, scan->header.stamp)) {
        return;
    }

    sensor_msgs::LaserScan laser_scan = *scan;
    for (unsigned int j = 0; j < laser_scan.ranges.size(); j++)
    {
        if (std::isnan(laser_scan.ranges[j]))
        {
            laser_scan.ranges[j] = laser_scan.range_max;
        }
    }

    // convert laser scan to point cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 scan_cloud;
    projector_.projectLaser(laser_scan, scan_cloud);
    pcl_df::fromROSMsg(scan_cloud, *latest_cloud);

    const Eigen::Matrix4f odom_to_scan_mat = odom_to_base_mat * base_to_scan_mat_;
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, odom_to_scan_mat);

    *fused_cloud_ptr_ += *latest_cloud;
    update_point_cloud_ = true;
}

void SensorDataProcess::depthCamCallback(const sensor_msgs::PointCloud2::ConstPtr& depth_cloud) {
    Eigen::Matrix4f odom_to_base_mat;
    nearst_time_ = depth_cloud->header.stamp;
    if (depth_cloud->data.size() == 0) {
        return;
    }
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_df::fromROSMsg(*depth_cloud, *latest_cloud);

    if(!pose_helper_->getOdomPose(odom_to_base_mat, depth_cloud->header.stamp)) {
        return;
    }
    
    const Eigen::Matrix4f odom_to_cam_mat = odom_to_base_mat * base_to_cam_mat_;
    pcl::transformPointCloud(*latest_cloud, *latest_cloud, odom_to_cam_mat);

    *fused_cloud_ptr_ += *latest_cloud;
}

void SensorDataProcess::processTimerCallback(const ros::TimerEvent& event) {
    // boost::lock_guard<boost::mutex> lock(point_cloud_mutex_);
    if (fused_cloud_ptr_->points.size() == 0)
        return;

    if (!update_point_cloud_) 
        return;


    if (fusion_point_pub_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 point_cloud;
        pcl::toROSMsg(*fused_cloud_ptr_, point_cloud);
        point_cloud.header.frame_id = "map";
        point_cloud.header.stamp = ros::Time::now();
        fusion_point_pub_.publish(point_cloud);
        fused_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    update_point_cloud_ = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_data_process");
    ros::NodeHandle nh("~");
    SensorDataProcess sensor_data_process(nh);

    ros::spin();
    return 0;
}