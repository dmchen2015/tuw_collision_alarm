//
// Created by firat on 31.07.19.
//

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tuw_collision_alarm");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "tuw_collision_alarm/CollisionAlarmNodelet", remap, nargv);


    ros::spin();
    return 0;
}