#include "cppad_bicycle_traj_opt/class_node_traj_opt.h"

#include <tf/tf.h>


namespace cppad_bicycle_traj_opt
{

ClassNodeTrajOpt::ClassNodeTrajOpt(ros::NodeHandle& nh) : nh_(nh)
{
    sparse_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sparse_path", 1);
    
    goal_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &ClassNodeTrajOpt::goalposeCallback, this);
    obstacles_sub_ = nh_.subscribe<visualization_msgs::InteractiveMarkerInit>("/basic_controls/update_full", 1, &ClassNodeTrajOpt::obstacleCallback, this);

    main_loop_timer_ = nh_.createTimer(ros::Duration(0.1), &ClassNodeTrajOpt::mainLoopCallback, this);
}

void ClassNodeTrajOpt::obstacleCallback(const visualization_msgs::InteractiveMarkerInit::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_obstacles_);
    obstacles_.clear();
    for (auto marker : msg->markers)
        obstacles_.push_back({marker.pose.position.x, marker.pose.position.y});
}

void ClassNodeTrajOpt::goalposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_goal_pose_);
    goal_pose_[0] = msg->pose.position.x;
    goal_pose_[1] = msg->pose.position.y;
    goal_pose_[2] = tf::getYaw(msg->pose.orientation);
}

void ClassNodeTrajOpt::mainLoopCallback(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> lock(mutex_obstacles_);
    std::lock_guard<std::mutex> lock_goal(mutex_goal_pose_);

    if (goal_pose_[0] == 0.0)
    {
        ROS_INFO("No goal pose received yet.");
        return;
    }

    size_t n_step = 20;
    double dt = 0.1;
    double Lf = 1.1;

    auto before_solve = std::chrono::high_resolution_clock::now();

    ClassTrajectorySolver traj_solver(n_step, dt, Lf, obstacles_, goal_pose_);
    
    auto after_solve = std::chrono::high_resolution_clock::now();
    auto time_solve = std::chrono::duration_cast<std::chrono::milliseconds>(after_solve - before_solve);
    std::cout << "Time to solve: " << time_solve.count() << "ms" << std::endl;

    auto sparse_path = traj_solver.getSolutionXYTheta();
    auto sparse_v = traj_solver.getSolutionV();

    // Publish the sparse path as RVIZ marker array
    visualization_msgs::MarkerArray sparse_path_msg;
    for (size_t i = 0; i < sparse_path.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "sparse_path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = sparse_path[i][0];
        marker.pose.position.y = sparse_path[i][1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(sparse_path[i][2]);
        marker.scale.x = std::abs(sparse_v[i]) / 15.0;
        marker.scale.y = 0.05;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        sparse_path_msg.markers.push_back(marker);
    }
    sparse_path_pub_.publish(sparse_path_msg);


}


} // namespace cppad_bicycle_traj_opt
