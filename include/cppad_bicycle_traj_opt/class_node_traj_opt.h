#ifndef CLASS_NODE_TRAJ_OPT_H
#define CLASS_NODE_TRAJ_OPT_H

#include <mutex>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/InteractiveMarkerInit.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"

#include <array>
#include <vector>
#include <chrono>

#include "cppad_bicycle_traj_opt/trajectory_solver.h"


namespace cppad_bicycle_traj_opt
{

class ClassNodeTrajOpt
{
public:
    ClassNodeTrajOpt(ros::NodeHandle& nh);

private:

    ros::NodeHandle nh_;

    ros::Publisher sparse_path_pub_;

    ros::Subscriber goal_pose_sub_;
    ros::Subscriber obstacles_sub_;

    ros::Timer main_loop_timer_;

    void goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void obstacleCallback(const visualization_msgs::InteractiveMarkerInit::ConstPtr& msg);

    void mainLoopCallback(const ros::TimerEvent&);

private:
    std::vector<std::array<double,2>> obstacles_;
    std::array<double,3> goal_pose_;  // x, y, theta_rad

    std::mutex mutex_obstacles_;
    std::mutex mutex_goal_pose_;

};

}  // namespace cppad_bicycle_traj_opt


#endif // CLASS_NODE_TRAJ_OPT_H
