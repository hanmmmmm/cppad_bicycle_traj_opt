
#include <iostream>
#include "ros/ros.h"

#include "cppad_bicycle_traj_opt/class_node_traj_opt.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cppad_bicycle_traj_opt_node");
    ros::NodeHandle nh;

    cppad_bicycle_traj_opt::ClassNodeTrajOpt node_traj_opt(nh);

    ros::spin();

    return 0;
}



