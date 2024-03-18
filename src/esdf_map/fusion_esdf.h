#ifndef FUSION_ESDF
#define FUSION_ESDF

//
#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>

// data structure
#include <queue>
#include <vector>
#include <unordered_map>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>

// tf
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.h>

// Laser & PCL
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// Odom & Pose
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

// Map
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>

// message filter
#include <message_filters/subscriber.h>    // flag buffers for speeding up raycasting
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// visual
#include <visualization_msgs/Marker.h>

#include <pcl_df_utils.h>
#include <get_pose_helper.h>

// spdlog
#include <spdlog/spdlog.h>

#define logit(x) (log((x) / (1 - (x))))

struct MappingParameters
{
    /* map properties */
    // map origin in pos
    Eigen::Vector2d map_origin_, local_map_origin_;
    // global map size
    Eigen::Vector2d map_size_;
    // global map range
    Eigen::Vector2d map_min_boundary_, map_max_boundary_;
    // map size in pixel
    Eigen::Vector2i map_pixel_num_, local_map_pixel_num_;  
    // map index                 
    Eigen::Vector2i map_min_idx_, map_max_idx_, local_min_idx_, local_max_idx_;
    // local map range
    Eigen::Vector2d local_x_range_, local_y_range_;
    // local pixel number in x and y
    Eigen::Vector2i local_index_range_;
    // local map relolution
    double resolution_, resolution_inv_;
    // obstacle inflation based on robot size
    double obstacles_inflation_;

    // frame id
    std::string odom_,odom_carto_;
    std::string frame_id_, base_id_, odom_id_, pointcloud_sensor_frame_id_, scan_sensor_frame_id_;

    /* raycasting */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit of occupancy probability
    /* active mapping */
    double unknown_flag_;

    /* local map update and clear */
    double z_min_, z_max_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MappingData
{
    // main map data, occupancy of each voxel and Euclidean distance
    int buffer_size_, local_buffer_size_;

    // global map
    // static occupancy map
    std::vector<char> occupancy_buffer_static_inflate_;
    std::vector<double> distance_buffer_static_all_;

    // local dynamic occupancy map
    // occupancy map(local) and distance map(local)
    std::vector<double>  local_occupancy_buffer_, local_distance_buffer_;
    // inflated occupancy map(local) and used for compute positive DT
    // used for compute negative distance
    std::vector<char> local_occupancy_buffer_inflate_, local_occupancy_buffer_neg_;
    std::vector<double> local_tmp_buffer_, local_distance_buffer_neg_, local_distance_buffer_all_;

    // laser position and pose data
    Eigen::Vector2d robot_pos_, last_robot_pos_;
    Eigen::Quaterniond robot_q_, last_robot_q_;

    // pointCloud2 data
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> fused_cloud_ptr_;

    // flags of map state
    bool has_odom_, has_static_map_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FusionMap
{
public:
    FusionMap() {}
    ~FusionMap() {}
    typedef std::shared_ptr<FusionMap> Ptr;

    enum
    {
        INVALID_IDX = -10000
    };

    void initMap(ros::NodeHandle &nh);

    geometry_msgs::PoseArray grad_pos_arr_;

    /* occupancy map management */
    // static map
    void get_static_buffer(std::vector<char> &static_buffer_inflate);

    // pos[meter] to index [pixel cell]
    inline void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);
    inline void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);
    inline Eigen::Vector2i posToIndex(const Eigen::Vector2d &pos);
    inline void localIndexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);
    inline void localPosToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);

    // index [pixel cell] to buffer array id [num]
    inline int toAddress(const Eigen::Vector2i &id);
    inline int toAddress(int &x, int &y);
    inline int toLocalAddress(const Eigen::Vector2i &id);
    inline int toLocalAddress(int &x, int &y);

    // is in map
    inline bool isInMap(const Eigen::Vector2d &pos);
    inline bool isInMap(const Eigen::Vector2i &idx);
    inline bool isInLocalMap(const Eigen::Vector2d &point);
    inline bool isInLocalMap(const Eigen::Vector2i &idx);

    // occupancy manage
    inline int getLocalInflateOccupancy(Eigen::Vector2d pos);
    inline int getLocalInflateOccupancy(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    inline int getStaticInflateOccupancy(Eigen::Vector2d pos); // fuse local and static map

    inline int getFusedDynamicInflateOccupancy(Eigen::Vector2d pos); // fuse local and static map and dynamic obs
    inline int getFusedDynamicInflateOccupancyi(Eigen::Vector2i index);
    inline double getFusedDynamicDistance(Eigen::Vector2d pos);
    inline double getFusedDynamicDistancei(Eigen::Vector2i index);

    // utils: bound index, known, unknown, free, occupied
    inline void boundIndex(Eigen::Vector2i &id);
    inline void boundLocalIndex(Eigen::Vector2i &id);
    inline bool isKnownOccupied(const Eigen::Vector2i &id);

    /* distance field management */
    // get distance
    inline double getDistance(const Eigen::Vector2d &pos);
    inline double getDistance(const Eigen::Vector2i &id);
    inline double getDistanceStatic(const Eigen::Vector2d &pos);
    inline double getDistanceDynamic(const Eigen::Vector2d &pos);
    inline double getDistanceDynamic(const Eigen::Vector2i &id);

    // get distance gradient
    void evaluateEDTBoth(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad);
    void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad);
    void evaluateLocalEDT(const Eigen::Vector2d &pos, double &dist, Eigen::Vector2d &grad);
    void debugEvalue(const std::vector<Eigen::Vector2d> &pos, const std::vector<double> &dist, const std::vector<Eigen::Vector2d> &grad);
    
    void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff);
    void getLocalSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff);
    
    void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
    void getLocalSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
    
    void interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                             double &value, Eigen::Vector2d &grad);

    void processFusedPointCloud();

    /* utils map */
    void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size);
    double getResolution();
    /* mutex  */
    void unlock();
    void lock();
    /* visualization publish */
    void publishStaticMap();
    void publishStaticESDF();
    void publishDynamicESDF();
    void publishDynamicMap();

private:
    ros::NodeHandle node_;
    MappingParameters mp_;
    MappingData md_;

    // scan to pointCloud2 projector
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr_;
    geometry_msgs::TransformStamped::Ptr TF_base_to_scan_ptr_;
    geometry_msgs::TransformStamped::Ptr odom_transform_ptr_;

    // sensor: subscriber
    ros::Subscriber async_odom_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tfL_;

    std::unique_ptr<pose_utils::GetPoseHelper> pose_helper_;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> fused_point_cloud_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> fused_point_cloud_filter_;

    // map server service
    ros::ServiceClient static_map_client_;
    nav_msgs::OccupancyGrid static_map_;

    // local map point cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_pointcloud_;

    // publiser
    ros::Publisher static_map_pub_, local_map_pub_, local_esdf_pub_;
    ros::Publisher static_esdf_pub_;
    ros::Publisher debug_pub_, debug_grad_pub_;


    Eigen::Matrix4f base_to_scan_mat_;
    Eigen::Matrix4f base_to_cam_mat_;
    bool update_point_cloud_;

    std::mutex odom_update_mutex_;
    std::mutex process_mutex_;

    /* Sensor Callbacks */
    void fusedPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);

    void visUpdate();
    void odomUpdate();

    // main update process
    /* occupancy map update */
    void projectPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);
    void preprocPointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);

    inline void inflatePoint(const Eigen::Vector2i &pt, int step, std::vector<Eigen::Vector2i> &pts);

    Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &robot_pos);

    //  service static callback
    bool get_static_map();

    /* ESDF map update */
    void updateESDF2d_static(std::vector<char> &occ_buffer_inflate, std::vector<double> &dist_buffer_all);
    void updateESDF2d_dynamic();

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
};

inline void FusionMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id)
{
    for (int i = 0; i < 2; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void FusionMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos)
{
    for (int i = 0; i < 2; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void FusionMap::localPosToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id)
{
    for (int i = 0; i < 2; ++i)
        id(i) = floor((pos(i) - mp_.local_map_origin_(i)) * mp_.resolution_inv_);
}

inline void FusionMap::localIndexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos)
{
    for (int i = 0; i < 2; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.local_map_origin_(i);
}

inline Eigen::Vector2i FusionMap::posToIndex(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    for (int i = 0; i < 2; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    return id;
}

inline int FusionMap::toAddress(const Eigen::Vector2i &id)
{
    return id(0) * mp_.map_pixel_num_(1) + id(1);
}

inline int FusionMap::toAddress(int &x, int &y)
{
    return x * mp_.map_pixel_num_(1) + y;
}

inline int FusionMap::toLocalAddress(const Eigen::Vector2i &id)
{
    return id(1) * mp_.local_map_pixel_num_(0) + id(0);
}

inline int FusionMap::toLocalAddress(int &x, int &y){
    return y * mp_.local_map_pixel_num_(0) + x;
}

inline bool FusionMap::isInMap(const Eigen::Vector2d &pos)
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
    {
        // cout << "less than min range!" << endl;
        return false;
    }

    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
    {
        return false;
    }

    return true;
}

inline bool FusionMap::isInMap(const Eigen::Vector2i &idx)
{
    if (idx(0) < 0 || idx(1) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.map_pixel_num_(0) - 1 || idx(1) > mp_.map_pixel_num_(1) - 1)
    {
        return false;
    }
    return true;
}

inline bool FusionMap::isInLocalMap(const Eigen::Vector2i &idx){
    if (idx(0) < 0 || idx(1) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.local_index_range_(0) - 1 || idx(1) > mp_.local_index_range_(1) - 1)
    {
        return false;
    }
    return true;
}

inline bool FusionMap::isInLocalMap(const Eigen::Vector2d &pos){
    if (pos(0) < mp_.local_x_range_(0) || pos(1) < mp_.local_y_range_(0))
    {
        return false;
    }
    if (pos(0) > mp_.local_x_range_(1) || pos(1) > mp_.local_y_range_(1))
    {
        return false;
    }
    return true;
}


inline int FusionMap::getLocalInflateOccupancy(Eigen::Vector2d pos){
    if (!isInLocalMap(pos))
        return -1;

    Eigen::Vector2i id;
    localPosToIndex(pos, id);
    return int(md_.local_occupancy_buffer_inflate_[toLocalAddress(id)]);
}

inline int FusionMap::getLocalInflateOccupancy(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){
    Eigen::Matrix4f odom_to_base_matrix;
    {
        std::lock_guard<std::mutex> lock(odom_update_mutex_);
        odom_to_base_matrix = tf2::transformToEigen(*(odom_transform_ptr_)).matrix().cast<float>();
    } 
    pcl::transformPointCloud(*point_cloud, *point_cloud, odom_to_base_matrix.inverse());

    for(size_t i = 1; i < point_cloud->points.size(); i++){
        if(!isInLocalMap(Eigen::Vector2d(point_cloud->points[i].x, point_cloud->points[i].y))){
            continue;
        }

        Eigen::Vector2i id;
        localPosToIndex(Eigen::Vector2d(point_cloud->points[i].x, point_cloud->points[i].y), id);
        if(md_.local_occupancy_buffer_[toLocalAddress(id)] > mp_.clamp_max_log_ - 0.1){
            ROS_ERROR("[Local map] Point %ld is occupied!", i);
            return i;
        }
    }
    return -1;
}

inline int FusionMap::getStaticInflateOccupancy(Eigen::Vector2d pos)
{
    /* 
     *  Note: Get the static map occupancy with inflation
     */
    if (!isInMap(pos))
        return 0;

    Eigen::Vector2i id;
    posToIndex(pos, id);

    if (md_.has_static_map_ && md_.occupancy_buffer_static_inflate_[toAddress(id)] == 1)
    {
        return 1;
    }

    return 0;
}

inline int FusionMap::getFusedDynamicInflateOccupancy(Eigen::Vector2d pos)
{
    /*
     *  Note: Note only check the local map occupancy, but also check the static map occupancy
     */
    if (!isInMap(pos))
        return 0;

    Eigen::Vector2i id;
    posToIndex(pos, id);

    if (md_.has_static_map_ && md_.occupancy_buffer_static_inflate_[toAddress(id)] == 1)
    {
        return 1;
    }

    Eigen::Matrix4f odom_to_base_matrix;
    {
        std::lock_guard<std::mutex> lock(odom_update_mutex_);
        odom_to_base_matrix = tf2::transformToEigen(*(odom_transform_ptr_)).matrix().cast<float>();
    } 
    Eigen::Vector4f pos4(pos(0), pos(1), 0, 1);
    pos4 = odom_to_base_matrix.inverse() * pos4;
    pos(0) = pos4(0);
    pos(1) = pos4(1);

    if (!isInLocalMap(pos))
        return 0;

    localPosToIndex(pos, id);
    return int(md_.local_occupancy_buffer_inflate_[toLocalAddress(id)]);
}

inline int FusionMap::getFusedDynamicInflateOccupancyi(Eigen::Vector2i index)
{
    Eigen::Vector2d pos;
    indexToPos(index, pos);
    return getFusedDynamicInflateOccupancy(pos);
}

inline double FusionMap::getFusedDynamicDistance(Eigen::Vector2d pos)
{
    if (!isInMap(pos))
        return 0.5;

    Eigen::Vector2i id;
    posToIndex(pos, id);

    if (md_.has_static_map_ && md_.occupancy_buffer_static_inflate_[toAddress(id)] == 1)
    {
        return 0.5;
    }

    Eigen::Matrix4f odom_to_base_matrix;
    {
        std::lock_guard<std::mutex> lock(odom_update_mutex_);
        odom_to_base_matrix = tf2::transformToEigen(*(odom_transform_ptr_)).matrix().cast<float>();
    } 
    Eigen::Vector4f pos4(pos(0), pos(1), 0, 1);
    pos4 = odom_to_base_matrix.inverse() * pos4;
    pos(0) = pos4(0);
    pos(1) = pos4(1);

    if (!isInLocalMap(pos))
        return 0.5;

    localPosToIndex(pos, id);
    return md_.local_distance_buffer_all_[toLocalAddress(id)];
}

inline double FusionMap::getFusedDynamicDistancei(Eigen::Vector2i index)
{
    Eigen::Vector2d pos;
    indexToPos(index, pos);
    return getFusedDynamicDistance(pos);
}

inline void FusionMap::boundIndex(Eigen::Vector2i &id)
{
    Eigen::Vector2i id1;
    id1(0) = std::max(std::min(id(0), mp_.map_pixel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.map_pixel_num_(1) - 1), 0);
    id = id1;
}

inline void FusionMap::boundLocalIndex(Eigen::Vector2i &id)
{
    Eigen::Vector2i id1;
    id1(0) = std::max(std::min(id(0), mp_.local_map_pixel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.local_map_pixel_num_(1) - 1), 0);
    id = id1;
}


inline bool FusionMap::isKnownOccupied(const Eigen::Vector2i &id)
{
    Eigen::Vector2i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    return md_.occupancy_buffer_static_inflate_[adr] == 1;
}

inline void FusionMap::inflatePoint(const Eigen::Vector2i &pt, int step, std::vector<Eigen::Vector2i> &pts)
{
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
        {
            pts[num++] = Eigen::Vector2i(pt(0) + x, pt(1) + y);
        }
}

/* DISTANCE FIELD*/
inline double FusionMap::getDistance(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex(pos, id);
    boundIndex(id);

    // return std::min(md_.distance_buffer_all_[toAddress(id)], md_.distance_buffer_static_all_[toAddress(id)]);
    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double FusionMap::getDistance(const Eigen::Vector2i &id)
{
    Eigen::Vector2i id1 = id;
    boundIndex(id1);
    // return std::min(md_.distance_buffer_all_[toAddress(id)], md_.distance_buffer_static_all_[toAddress(id)]);
    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double FusionMap::getDistanceStatic(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex(pos, id);
    boundIndex(id);

    return md_.distance_buffer_static_all_[toAddress(id)];
}

inline double FusionMap::getDistanceDynamic(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    localPosToIndex(pos, id);

    return md_.local_distance_buffer_all_[toLocalAddress(id)];
}

inline double FusionMap::getDistanceDynamic(const Eigen::Vector2i &id)
{
    Eigen::Vector2i id1 = id;
    boundLocalIndex(id1);
    return md_.local_distance_buffer_all_[toLocalAddress(id1)];
}
#endif