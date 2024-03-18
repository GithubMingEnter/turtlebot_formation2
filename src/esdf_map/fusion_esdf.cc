#include "fusion_esdf.h"

void FusionMap::initMap(ros::NodeHandle &nh)
{
    node_ = nh;

    /* get static map's parameters*/
    std::string static_map_service_name = "/static_map";
    ros::service::waitForService(static_map_service_name); // important
    static_map_client_ = nh.serviceClient<nav_msgs::GetMap>(static_map_service_name);
    bool is_static_map_avail = get_static_map();
    if (!is_static_map_avail)
    {
        ROS_ERROR("No static map available, please check map_server.");
    }

    // local map size
    node_.param("local_map/local_update_range_min_x", mp_.local_x_range_(0), -2.0);
    node_.param("local_map/local_update_range_max_x", mp_.local_x_range_(1), 8.0);
    node_.param("local_map/local_update_range_min_y", mp_.local_y_range_(0), -5.0);
    node_.param("local_map/local_update_range_max_y", mp_.local_y_range_(1), 5.0);

    // occupancy map
    node_.param("local_map/p_hit", mp_.p_hit_, 0.55);
    node_.param("local_map/p_miss", mp_.p_miss_, 0.2);
    node_.param("local_map/p_min", mp_.p_min_, 0.12);
    node_.param("local_map/p_max", mp_.p_max_, 0.80);
    node_.param("local_map/p_occ", mp_.p_occ_, 0.70);


    node_.param("local_map/odom", mp_.odom_, std::string("odom"));
    node_.param("local_map/odom_carto", mp_.odom_carto_, std::string("odom_carto"));

    node_.param("local_map/frame_id", mp_.frame_id_, std::string("map"));
    node_.param("local_map/base_id", mp_.base_id_, std::string("base_footprint"));
    node_.param("local_map/pointcloud_sensor_frame_id", mp_.pointcloud_sensor_frame_id_, std::string("base_link"));
    node_.param("local_map/scan_sensor_frame_id", mp_.scan_sensor_frame_id_, std::string("base_scan"));


    spdlog::info("[Local map]: mp_.base_id_{}", mp_.base_id_);
    spdlog::info("[Local map]: mp_.scan_sensor_frame_id_ {}", mp_.scan_sensor_frame_id_);
    spdlog::info("[Local map]: mp_.pointcloud_sensor_frame_id_ {}", mp_.pointcloud_sensor_frame_id_);
    // local map
    node_.param("local_map/obstacles_inflation", mp_.obstacles_inflation_, 0.15);
    node_.param("local_map/z_threshold", mp_.z_min_, 0.01); // only consider the point greater than z_threshold_
    node_.param("local_map/z_threshold", mp_.z_max_, 0.01); // only consider the point greater than z_threshold_

    /* map size*/
    // resolution
    mp_.resolution_ = static_map_.info.resolution;
    mp_.resolution_inv_ = 1 / mp_.resolution_;

    // size in meter
    double x_size, y_size, x_origin, y_origin;
    x_size = static_map_.info.width * static_map_.info.resolution;  // width in meter
    y_size = static_map_.info.height * static_map_.info.resolution; // height in meter
    mp_.map_size_ = Eigen::Vector2d(x_size, y_size);                // in meter

    x_origin = static_map_.info.origin.position.x;         // coordinate of left-bottom corner of the map, in meter
    y_origin = static_map_.info.origin.position.y;         // coordinate of left-bottom corner of the map, in meter
    mp_.map_origin_ = Eigen::Vector2d(x_origin, y_origin); // left-bottom corner  w.r.t coordinate origin

    // size in pixel of global map
    // for (int i = 0; i < 2; ++i) mp_.map_pixel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    mp_.map_pixel_num_(0) = static_map_.info.width;  // pixel num: width
    mp_.map_pixel_num_(1) = static_map_.info.height; // pixel num: height

    // global map boundary in pixel/index
    mp_.map_min_idx_ = Eigen::Vector2i::Zero();                      // min pixel idx  (x_pixel_min,y_pixel_min)
    mp_.map_max_idx_ = mp_.map_pixel_num_ - Eigen::Vector2i::Ones(); // max pixel idx  (x_pixel_max,y_pixel_max)

    // global map boundary in meter
    mp_.map_min_boundary_ = mp_.map_origin_; // map boundary (x_min,y_min) in meter
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    // local map boundary in pixel/index
    mp_.local_map_origin_ = Eigen::Vector2d(mp_.local_x_range_(0), mp_.local_y_range_(0)); // local map origin (x_min,y_min) in meter
    mp_.local_index_range_ = Eigen::Vector2i(ceil((mp_.local_x_range_(1) - mp_.local_x_range_(0)) / mp_.resolution_),
                                             ceil((mp_.local_y_range_(1) - mp_.local_y_range_(0)) / mp_.resolution_));
    mp_.local_map_pixel_num_ = mp_.local_index_range_;
    mp_.local_min_idx_ = Eigen::Vector2i::Zero();                            // min pixel idx  (x_pixel_min,y_pixel_min)
    mp_.local_max_idx_ = mp_.local_map_pixel_num_ - Eigen::Vector2i::Ones(); // max pixel idx  (x_pixel_max,y_pixel_max)

    // occupancy map probability param                    // odd(s+)=odd(s-)+prob_hit_log_ or odd(s+)=odd(s-)+likelihood
    mp_.prob_hit_log_ = logit(mp_.p_hit_);      // log likelihood log[p(z=hit|s=occ)/p(m=hit|s=free)]
    mp_.prob_miss_log_ = logit(mp_.p_miss_);    // log likelihood log[p(z=miss|s=occ)/p(m=miss|s=free)]
    mp_.clamp_min_log_ = logit(mp_.p_min_);     // log min state prob  log[p(s=occ)/p(s=free)]
    mp_.clamp_max_log_ = logit(mp_.p_max_);     // log max state prob  log[p(s=occ)/p(s=free)]
    mp_.min_occupancy_log_ = logit(mp_.p_occ_); // log of occupancy determined prob
    mp_.unknown_flag_ = 0.01;


    md_.has_odom_ = false; // odom callback [not much use]
    md_.has_static_map_ = false;

    /* initialize data buffers*/
    // global map buffer size
    md_.buffer_size_ = mp_.map_pixel_num_(0) * mp_.map_pixel_num_(1);                   // buffer size
    md_.local_buffer_size_ = mp_.local_map_pixel_num_(0) * mp_.local_map_pixel_num_(1); // local buffer size

    // global occupancy map buffer
    // save state_occu probability [mp_.clamp_min_log_,mp_.clamp_max_log_]
    // md_.occupancy_buffer_ = std::vector<double>(md_.buffer_size_, mp_.clamp_min_log_ - mp_.unknown_flag_); (not use)
    // md_.occupancy_buffer_neg_ = std::vector<char>(md_.buffer_size_, 0);
    md_.local_occupancy_buffer_ = std::vector<double>(md_.local_buffer_size_, mp_.clamp_min_log_ - mp_.unknown_flag_);
    md_.local_occupancy_buffer_neg_ = std::vector<char>(md_.local_buffer_size_, 0);
    md_.local_occupancy_buffer_inflate_ = std::vector<char>(md_.local_buffer_size_, 0);

    md_.occupancy_buffer_static_inflate_ = std::vector<char>(md_.buffer_size_, 0); // static map buffer

    // global distance map buffer
    md_.distance_buffer_static_all_ = std::vector<double>(md_.buffer_size_, 10000);

    // local distance map buffer
    md_.local_distance_buffer_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_distance_buffer_neg_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_distance_buffer_all_ = std::vector<double>(md_.local_buffer_size_, 0);
    md_.local_tmp_buffer_ = std::vector<double>(md_.local_buffer_size_, 0);

    // local occupancy point cloud
    md_.fused_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // init static occ & ESDF buffers
    get_static_buffer(md_.occupancy_buffer_static_inflate_);
    updateESDF2d_static(md_.occupancy_buffer_static_inflate_, md_.distance_buffer_static_all_);

    /* show map param */
    spdlog::info("[Local map]: X_size: {:.2f}", x_size);
    spdlog::info("[Local map]: Y_size: {:.2f}", y_size);
    spdlog::info("[Local map]: X_origin: {:.2f}", x_origin);
    spdlog::info("[Local map]: Y_origin: {:.2f}", y_origin);
    spdlog::info("[Local map]: X_size_pix: {:d}", mp_.map_pixel_num_(0));
    spdlog::info("[Local map]: Y_size_pix: {:d}", mp_.map_pixel_num_(1));
    spdlog::info("[Local map]: X_map_max_idx: {:d}", mp_.map_max_idx_(0));
    spdlog::info("[Local map]: Y_map_max_idx: {:d}", mp_.map_max_idx_(1));
    spdlog::info("[Local map]: Hit:  {:.2f}", mp_.prob_hit_log_);
    spdlog::info("[Local map]: Miss: {:.2f}", mp_.prob_miss_log_);
    spdlog::info("[Local map]: Min_log: {:.2f}", mp_.clamp_min_log_);
    spdlog::info("[Local map]: Max_log: {:.2f}", mp_.clamp_max_log_);
    spdlog::info("[Local map]: Threshold log: {:.2f}", mp_.min_occupancy_log_);

    ros::NodeHandle public_nh("");

    update_point_cloud_ = false;

    // debug
    debug_pub_      = public_nh.advertise<sensor_msgs::PointCloud2>("debug/points", 1);
    debug_grad_pub_ = public_nh.advertise<geometry_msgs::PoseArray>("debug/grad", 1);

    // Publishers
    // local map and esdf map pub
    local_map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/localOccupancyMap", 10);
    local_esdf_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/localEsdfMap", 10);

    // global map and esdf map pub
    static_map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/globalOccupancyMap", 10);
    static_esdf_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("local_map/globalEsdfMap", 10);

    TF_base_to_sensor_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    TF_base_to_scan_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    odom_transform_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);

    tf2_ros::Buffer tfBuffer_scan, tfBuffer_pc;
    tf2_ros::TransformListener tfListener_scan(tfBuffer_scan), tfListener_pc(tfBuffer_pc);

    try
    {
        *(TF_base_to_scan_ptr_) = tfBuffer_scan.lookupTransform(mp_.base_id_, mp_.scan_sensor_frame_id_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        spdlog::warn("fail to listen the transform of {}->{}. Error message: {}", mp_.base_id_.c_str(), mp_.scan_sensor_frame_id_.c_str(), ex.what());
        return;
    }
    try
    {
        *(TF_base_to_sensor_ptr_) = tfBuffer_pc.lookupTransform(mp_.base_id_, mp_.pointcloud_sensor_frame_id_, ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        spdlog::warn("fail to listen the transform of {}->{}. Error message: {}", mp_.base_id_.c_str(), mp_.pointcloud_sensor_frame_id_.c_str(), ex.what());
        return;
    }
    base_to_cam_mat_ = tf2::transformToEigen(*(TF_base_to_sensor_ptr_)).matrix().cast<float>();
    base_to_scan_mat_ = tf2::transformToEigen(*(TF_base_to_scan_ptr_)).matrix().cast<float>();


    tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(1.0));
    tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(tf_.get(), mp_.base_id_, mp_.odom_carto_);

    // visualization update thread
    std::unique_ptr<boost::thread> update_odom = std::make_unique<boost::thread>(boost::bind(&FusionMap::odomUpdate, this));
    std::unique_ptr<boost::thread> update_thread = std::make_unique<boost::thread>(boost::bind(&FusionMap::visUpdate, this));

    fused_point_cloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(public_nh, "/fused_point_cloud", 2);
    fused_point_cloud_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>>(*fused_point_cloud_sub_, *tf_, mp_.odom_carto_, 2, public_nh);
    fused_point_cloud_filter_->registerCallback(boost::bind(&FusionMap::fusedPointCloudCallback, this, _1));
}

/* check if have static map */
bool FusionMap::get_static_map()
{
    nav_msgs::GetMap srv;
    srv.request = {}; // std_srvs::Empty::Request& request

    if (static_map_client_.call(srv))
    {
        spdlog::info("[Local map]: Is stacic map available: True" );
        // set global_plan
        static_map_ = srv.response.map;
        return true;
    }
    else
    {
        spdlog::error("[Local map]: Is stacic map available: False");
        return false;
    }
}

/* Map utils */
void FusionMap::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size)
{
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

double FusionMap::getResolution()
{
    return mp_.resolution_;
}


void FusionMap::fusedPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud) {
    Eigen::Matrix4f odom_to_base_mat;
    
    if (cloud->data.size() == 0) 
        return;

    // std::cout << "before update" << std::endl;
    if(!pose_helper_->getOdomPose(odom_to_base_mat, cloud->header.stamp-ros::Duration(0.1)))
        return;

    // std::cout << "update  local map" << std::endl;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_df::fromROSMsg(*cloud, *latest_cloud);

    local_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*latest_cloud, *local_pointcloud_, odom_to_base_mat.inverse());

    boost::thread process_thread(&FusionMap::processFusedPointCloud, this);
}
void FusionMap::lock(){
    process_mutex_.lock();
}

void FusionMap::unlock()
{
    process_mutex_.unlock();
}

void FusionMap::processFusedPointCloud()
{
    std::unique_lock<std::mutex> process_lock(process_mutex_);
    std::fill(md_.local_occupancy_buffer_.begin(), md_.local_occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
    std::fill(md_.local_occupancy_buffer_inflate_.begin(), md_.local_occupancy_buffer_inflate_.end(), 0);
    std::fill(md_.local_occupancy_buffer_neg_.begin(), md_.local_occupancy_buffer_neg_.end(), 0);

    preprocPointCloud(local_pointcloud_);
    projectPointCloud(local_pointcloud_);

    // publish the local_pointcloud_ on debug topic
    updateESDF2d_dynamic();
    process_lock.unlock();
    
    if (debug_pub_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 debug_cloud;
        pcl::toROSMsg(*local_pointcloud_, debug_cloud);
        debug_cloud.header.frame_id = "base_link";
        debug_cloud.header.stamp = ros::Time::now();
        debug_pub_.publish(debug_cloud);
    }
}

void FusionMap::preprocPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud)
{
    // apply a passthrough filter to the point cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    // count time 
    
    pass.setInputCloud(point_cloud);
    // filter the point cloud at a height
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.05, 0.3);
    pass.filter(*point_cloud);
    // filter the point cloud at a distance in x
    pass.setFilterFieldName("x");
    pass.setFilterLimits(mp_.local_x_range_(0), mp_.local_x_range_(1));
    pass.filter(*point_cloud);
    // filter the point cloud at a distance in y
    pass.setFilterFieldName("y");
    pass.setFilterLimits(mp_.local_y_range_(0), mp_.local_y_range_(1));
    pass.filter(*point_cloud);
}

void FusionMap::projectPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud)
{
    pcl::PointXYZ pt;
    Eigen::Vector2d p2d, p2d_inf;
    Eigen::Vector2i inf_pt;

    double inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    // double inf_step = 0;

    for (size_t i = 0; i < point_cloud->points.size(); ++i)
    {
        // current point
        pt = point_cloud->points[i];
        p2d(0) = pt.x, p2d(1) = pt.y;
        // convert to index

        if (!isInLocalMap(p2d))
            continue;

        for (int x = -inf_step; x <= inf_step; ++x)
        {
            for (int y = -inf_step; y <= inf_step; ++y)
            {
                p2d_inf(0) = p2d(0) + x * mp_.resolution_;
                p2d_inf(1) = p2d(1) + y * mp_.resolution_;
                if (!isInLocalMap(p2d_inf))
                    continue;

                localPosToIndex(p2d_inf, inf_pt);
                md_.local_occupancy_buffer_[toLocalAddress(inf_pt)] = mp_.clamp_max_log_;
                md_.local_occupancy_buffer_inflate_[toLocalAddress(inf_pt)] = 1; // is used to construct the local distance map
            }
        }
    }
}

void FusionMap::odomUpdate()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        Eigen::Matrix4f odom_to_base_mat;

        if(!pose_helper_->getOdomPose(odom_to_base_mat, ros::Time::now()-ros::Duration(0.1)))
            continue;
        
        {
            std::lock_guard<std::mutex> lock(odom_update_mutex_);
            odom_transform_ptr_->transform.translation.x = odom_to_base_mat(0, 3);
            odom_transform_ptr_->transform.translation.y = odom_to_base_mat(1, 3);
            odom_transform_ptr_->transform.translation.z = odom_to_base_mat(2, 3);
            Eigen::Quaternionf q(odom_to_base_mat.block<3, 3>(0, 0));
            odom_transform_ptr_->transform.rotation.x = q.x();
            odom_transform_ptr_->transform.rotation.y = q.y();
            odom_transform_ptr_->transform.rotation.z = q.z();
            odom_transform_ptr_->transform.rotation.w = q.w();
        }

        rate.sleep();
    }
}

void FusionMap::evaluateEDTBoth(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad)
{
    Eigen::Vector2d global_grad, local_grad;
    double global_dist, local_dist;

    evaluateEDTWithGrad(pos, global_dist, global_grad);
    evaluateLocalEDT(pos, local_dist, local_grad);

    if(global_dist < local_dist)
    {
        dist = global_dist;
        grad = global_grad;
    }
    else
    {
        dist = local_dist;
        grad = local_grad;
    }
}

void FusionMap::evaluateLocalEDT(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad)
{
    /*
     * @brief Generate the distance and gradient of the distance at the given position in local.
     */
    // transform pos point to local point
    
    Eigen::Matrix4f odom_to_base_matrix;
    {
        std::lock_guard<std::mutex> lock(odom_update_mutex_);
        odom_to_base_matrix = tf2::transformToEigen(*(odom_transform_ptr_)).matrix().cast<float>();
    } 
    Eigen::Vector4f transform_point = Eigen::Vector4f::Zero();
    transform_point.head<2>() = pos.cast<float>();
    transform_point(2) = 0.0;
    transform_point(3) = 1.0;
    transform_point =  odom_to_base_matrix.inverse() * transform_point;
    Eigen::Vector2d local_pos = transform_point.head<2>().cast<double>();

    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getLocalSurroundPts(local_pos, sur_pts, diff);
    
    Eigen::Vector2i local_id;
    localPosToIndex(local_pos, local_id);

    if(!isInLocalMap(local_id))
    {
        dist = 0;
        grad = Eigen::Vector2d::Zero();
        return;
    }

    // get distances of the surround pts
    double dists[2][2];
    getLocalSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateBilinear(dists, diff, dist, grad);

    transform_point.head<2>() = grad.cast<float>();
    // let odom_to_base_matrix translation to be zero
    odom_to_base_matrix.block<2, 1>(0, 3) = Eigen::Vector2f::Zero();
    transform_point = odom_to_base_matrix * transform_point;
    grad = transform_point.head<2>().cast<double>();
}

void FusionMap::debugEvalue(const std::vector<Eigen::Vector2d> &pos, const std::vector<double> &dist, 
                          const std::vector<Eigen::Vector2d> &grad)
{
    grad_pos_arr_.poses.clear();
    grad_pos_arr_.header.frame_id = "map";
    grad_pos_arr_.header.stamp = ros::Time::now();
    
    assert(pos.size() == dist.size() && pos.size() == grad.size());
    for(size_t i=0; i < pos.size(); i++){
        geometry_msgs::Pose pose;
        pose.position.x = pos[i](0);
        pose.position.y = pos[i](1);
        pose.position.z = 0.0;
        double angle = atan2(grad[i](1), grad[i](0));
        pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        grad_pos_arr_.poses.push_back(pose);
    }
    
    debug_grad_pub_.publish(grad_pos_arr_);
}

void FusionMap::getLocalSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
                             Eigen::Vector2d &diff)
{
    Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx;
    Eigen::Vector2d idx_pos;

    localPosToIndex(pos_m, idx);
    localIndexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_;

    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            Eigen::Vector2d current_pos;
            localIndexToPos(current_idx, current_pos);
            pts[x][y] = current_pos;
        }
    }
}

void FusionMap::getLocalSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
{
    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            dists[x][y] = getDistanceDynamic(pts[x][y]);
        }
    }
}

void FusionMap::evaluateEDTWithGrad(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad)
{
    /*
     * @brief Generate the distance and gradient of the distance at the given position.
     */

    // get diff & surround pts
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(pos, sur_pts, diff);

    // get distances of the surround pts
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateBilinear(dists, diff, dist, grad);
}

void FusionMap::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
                             Eigen::Vector2d &diff)
{
    // if (!isInMap(pos)) { std::cout << "pos invalid for interpolation." << std::endl; }

    /* interpolation position */
    Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx;
    Eigen::Vector2d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_; // (p-p0)/ (p1-p0)

    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            Eigen::Vector2d current_pos;
            indexToPos(current_idx, current_pos);
            pts[x][y] = current_pos;
        }
    }
}

void FusionMap::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
{
    for (int x = 0; x < 2; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            dists[x][y] = getDistance(pts[x][y]);
        }
    }
}

void FusionMap::interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff, double &value, Eigen::Vector2d &grad)
{
    // bilinear interpolation
    double v00 = values[0][0]; // f(x0,y0)
    double v10 = values[1][0]; // f(x1,y0)
    double v01 = values[0][1]; // f(x0,y1)
    double v11 = values[1][1]; // f(x1,y1)

    double v0 = (1 - diff(0)) * v00 + diff(0) * v10;
    double v1 = (1 - diff(0)) * v01 + diff(0) * v11;
    value = (1 - diff(1)) * v0 + diff(1) * v1;

    // calculate gradient
    grad[1] = (v1 - v0) * mp_.resolution_inv_;
    grad[0] = ((1 - diff[1]) * (v10 - v00) + diff[1] * (v11 - v01)) * mp_.resolution_inv_;
}

void FusionMap::get_static_buffer(std::vector<char> &static_buffer_inflate)
{
    int idx_nav_occ;
    int data;
    double value;

    md_.has_static_map_ = true;
    /* inflate the point */
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

    for (int id_x = 0; id_x < mp_.map_pixel_num_(0); id_x++)
    {
        for (int id_y = 0; id_y < mp_.map_pixel_num_(1); id_y++)
        {
            // [*] addr in nav_msg::OccupancyGrid.data
            idx_nav_occ = id_x + id_y * mp_.map_pixel_num_(0); // cell_col_num+ cell_row_num*rows

            // [*] cast data from int8 to int[important]
            data = (int)static_map_.data[idx_nav_occ];

            // Check data not be 0 or 100
            if (data != 0 && data != 100)
            {
                // std::cout<<"data probability="<<value<<std::endl;
            }
            value = double(data) / 100.0;

            // save data to buffer
            if (value > mp_.p_occ_)
            {
                // [*] addr in buffer
                Eigen::Vector2i idx;
                Eigen::Vector2d idx_pos;

                idx(0) = id_x;
                idx(1) = id_y;
                indexToPos(idx, idx_pos);
                // idx_inf=toAddress(idx);
                // md_.occupancy_buffer_static_inflate_[idx_inf]=1;

                /* inflate the point */
                Eigen::Vector2i inf_pt;
                Eigen::Vector2d p2d_inf;

                // Determine local occupandcy boundary for current location
                double max_x, max_y, min_x, min_y;

                min_x = mp_.map_max_boundary_(0);
                min_y = mp_.map_max_boundary_(1);
                max_x = mp_.map_min_boundary_(0);
                max_y = mp_.map_min_boundary_(1);

                for (int x = -inf_step; x <= inf_step; ++x)
                    for (int y = -inf_step; y <= inf_step; ++y)
                    {
                        p2d_inf(0) = idx_pos(0) + x * mp_.resolution_;
                        p2d_inf(1) = idx_pos(1) + y * mp_.resolution_;

                        max_x = std::max(max_x, p2d_inf(0));
                        max_y = std::max(max_y, p2d_inf(1));
                        min_x = std::min(min_x, p2d_inf(0));
                        min_y = std::min(min_y, p2d_inf(1));

                        posToIndex(p2d_inf, inf_pt);

                        if (!isInMap(inf_pt))
                            continue;
                        int idx_inf = toAddress(inf_pt);
                        static_buffer_inflate[idx_inf] = 1;
                    }
            }
        } // end for
    }
}

void FusionMap::updateESDF2d_static(std::vector<char> &occ_buffer_inflate, std::vector<double> &dist_buffer_all)
{
    Eigen::Vector2i min_esdf = mp_.map_min_idx_;
    Eigen::Vector2i max_esdf = mp_.map_max_idx_;

    std::vector<char>   occ_buffer_neg = std::vector<char>(md_.buffer_size_, 0);
    std::vector<double> tmp_buffer = std::vector<double>(md_.buffer_size_, 0);
    std::vector<double> dist_buffer = std::vector<double>(md_.buffer_size_, 10000);
    std::vector<double> dist_buffer_neg = std::vector<double>(md_.buffer_size_, 10000);

    /* ========== compute positive DT ========== */
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF([&](int y)
                 { return occ_buffer_inflate[toAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max(); },
                 [&](int y, double val)
                 { tmp_buffer[toAddress(x, y)] = val; },
                 min_esdf[1],
                 max_esdf[1],
                 1);
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF([&](int x)
                 { return tmp_buffer[toAddress(x, y)]; },
                 [&](int x, double val)
                 { dist_buffer[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val); },
                 min_esdf[0],
                 max_esdf[0],
                 0);
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toAddress(x, y);
            if (occ_buffer_inflate[idx] == 0)
            {
                occ_buffer_neg[idx] = 1;
            }
            else if (occ_buffer_inflate[idx] == 1)
            {
                occ_buffer_neg[idx] = 0;
            }
            else
            {
                ROS_ERROR("what?");
            }
        }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF([&](int y)
                 { return occ_buffer_neg[x * mp_.map_pixel_num_(1) + y] == 1 ? 0 : std::numeric_limits<double>::max(); },
                 [&](int y, double val)
                 { tmp_buffer[toAddress(x, y)] = val; },
                 min_esdf[1],
                 max_esdf[1],
                 1);
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF([&](int x)
                 { return tmp_buffer[toAddress(x, y)]; },
                 [&](int x, double val)
                 { dist_buffer_neg[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val); },
                 min_esdf[0],
                 max_esdf[0],
                 0);
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toAddress(x, y);
            dist_buffer_all[idx] = dist_buffer[idx];

            if (dist_buffer_neg[idx] > 0.0)
                dist_buffer_all[idx] += (-dist_buffer_neg[idx] + mp_.resolution_);
        }
}

void FusionMap::updateESDF2d_dynamic()
{
    Eigen::Vector2i min_esdf = mp_.local_min_idx_;
    Eigen::Vector2i max_esdf = mp_.local_max_idx_;

    std::fill(md_.local_distance_buffer_.begin(), md_.local_distance_buffer_.end(), 10000); // default 10000
    std::fill(md_.local_tmp_buffer_.begin(), md_.local_tmp_buffer_.end(), 0);
    std::fill(md_.local_distance_buffer_neg_.begin(), md_.local_distance_buffer_neg_.end(), 10000);
    std::fill(md_.local_distance_buffer_all_.begin(), md_.local_distance_buffer_all_.end(), 10000);  // is the final result

    /* ========== compute positive DT ========== */
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF([&](int y)
                 { return md_.local_occupancy_buffer_inflate_[toLocalAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max(); }, // TODO
                 [&](int y, double val)
                 { md_.local_tmp_buffer_[toLocalAddress(x, y)] = val; },
                 min_esdf[1],
                 max_esdf[1],
                 1);
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF([&](int x)
                 { return md_.local_tmp_buffer_[toLocalAddress(x, y)]; },
                 [&](int x, double val)
                 { md_.local_distance_buffer_[toLocalAddress(x, y)] = mp_.resolution_ * std::sqrt(val); },
                 min_esdf[0],
                 max_esdf[0],
                 0);
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toLocalAddress(x, y);
            if (md_.local_occupancy_buffer_inflate_[idx] == 0)
            {
                md_.local_occupancy_buffer_neg_[idx] = 1;
            }
            else if (md_.local_occupancy_buffer_inflate_[idx] == 1)
            {
                md_.local_occupancy_buffer_neg_[idx] = 0;
            }
            else
            {
                ROS_ERROR("what?");
            }
        }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    {
        fillESDF([&](int y)
                 { return md_.local_occupancy_buffer_neg_[x * mp_.map_pixel_num_(1) + y] == 1 ? 0 : std::numeric_limits<double>::max(); },
                 [&](int y, double val)
                 { md_.local_tmp_buffer_[toLocalAddress(x, y)] = val; },
                 min_esdf[1],
                 max_esdf[1],
                 1);
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    {
        fillESDF([&](int x)
                 { return md_.local_tmp_buffer_[toLocalAddress(x, y)]; },
                 [&](int x, double val)
                 { md_.local_distance_buffer_neg_[toLocalAddress(x, y)] = mp_.resolution_ * std::sqrt(val); },
                 min_esdf[0],
                 max_esdf[0],
                 0);
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        {
            int idx = toLocalAddress(x, y);
            md_.local_distance_buffer_all_[idx] = md_.local_distance_buffer_[idx];

            if (md_.local_distance_buffer_neg_[idx] > 0.0)
                md_.local_distance_buffer_all_[idx] += (-md_.local_distance_buffer_neg_[idx] + mp_.resolution_);
        }
}

template <typename F_get_val, typename F_set_val>
void FusionMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
    int v[mp_.map_pixel_num_(dim)];
    double z[mp_.map_pixel_num_(dim) + 1];

    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++)
    {
        k++;
        double s;

        do
        {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++)
    {
        while (z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

Eigen::Vector2d FusionMap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &robot_pos)
{
    Eigen::Vector2d diff = pt - robot_pos;
    Eigen::Vector2d max_tc = mp_.map_max_boundary_ - robot_pos;
    Eigen::Vector2d min_tc = mp_.map_min_boundary_ - robot_pos;

    double min_t = 1000000;

    for (int i = 0; i < 2; ++i)
    {
        if (fabs(diff[i]) > 0)
        {

            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;

            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }

    return robot_pos + (min_t - 1e-3) * diff;
}

void FusionMap::publishDynamicMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < mp_.local_map_pixel_num_(0) * mp_.local_map_pixel_num_(1); ++i)
    {
        if (md_.local_occupancy_buffer_inflate_[i] == 1)
        {
            Eigen::Vector2d pos;
            localIndexToPos(Eigen::Vector2i(i % mp_.local_map_pixel_num_(0), i / mp_.local_map_pixel_num_(0)), pos);

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = 0; // z=0
            cloud.push_back(pt);
        }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;

    cloud.is_dense = true;
    cloud.header.frame_id = mp_.base_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    local_map_pub_.publish(cloud_msg);
}

void FusionMap::publishStaticMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int x = 0; x <= mp_.map_pixel_num_(0); ++x)
        for (int y = 0; y <= mp_.map_pixel_num_(1); ++y)
        {
            int idx_inf = toAddress(x, y);
            if (md_.occupancy_buffer_static_inflate_[idx_inf] == 0)
                continue;

            Eigen::Vector2d pos;
            indexToPos(Eigen::Vector2i(x, y), pos);

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = TF_base_to_scan_ptr_->transform.translation.z; // z=0
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;

    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    static_map_pub_.publish(cloud_msg);
}

void FusionMap::publishStaticESDF()
{
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector2i min_cut = mp_.map_min_idx_; // md_.local_bound_min_ - Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector2i max_cut = mp_.map_max_idx_; // md_.local_bound_max_ + Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        {

            Eigen::Vector2d pos;
            indexToPos(Eigen::Vector2i(x, y), pos);

            dist = getDistanceStatic(pos);
            dist = std::min(dist, max_dist);
            dist = std::max(dist, min_dist);

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = 0.0;
            pt.intensity = (dist - min_dist) / (max_dist - min_dist);
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    static_esdf_pub_.publish(cloud_msg);
}

void FusionMap::publishDynamicESDF()
{
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    for (int x = mp_.local_min_idx_(0); x <= mp_.local_max_idx_(0); ++x)
        for (int y = mp_.local_min_idx_(1); y <= mp_.local_max_idx_(1); ++y)
        {

            dist = getDistanceDynamic(Eigen::Vector2i(x, y));
            dist = std::min(dist, max_dist);
            dist = std::max(dist, min_dist);

            Eigen::Vector2d pos;
            localIndexToPos(Eigen::Vector2i(x, y), pos);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = 0.0;
            pt.intensity = (dist - min_dist) / (max_dist - min_dist);
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.base_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    local_esdf_pub_.publish(cloud_msg);
}


void FusionMap::visUpdate()
{
    ros::Rate r(10);

    while(ros::ok()){
        if (static_map_pub_.getNumSubscribers() > 0)
            publishStaticMap();
        if (static_esdf_pub_.getNumSubscribers() > 0)
            publishStaticESDF();
        if (local_map_pub_.getNumSubscribers() > 0)
            publishDynamicMap();
        if (local_esdf_pub_.getNumSubscribers() > 0)
            publishDynamicESDF();

        r.sleep();
    }
    
}
