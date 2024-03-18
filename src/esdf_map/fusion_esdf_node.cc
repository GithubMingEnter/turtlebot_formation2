#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "fusion_esdf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_esdf_map_node");
    ros::NodeHandle nh("~");

    FusionMap grid_map;

    grid_map.initMap(nh);
    ros::Duration(1.0).sleep();
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
