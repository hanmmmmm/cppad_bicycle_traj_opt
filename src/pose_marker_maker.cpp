
#include <vector>

#include "cppad_bicycle_traj_opt/pose_marker_maker.h"


namespace cppad_bicycle_traj_opt
{


std::vector<std::array<geometry_msgs::Point, 2>> makeArrow(const double x, const double y, const double yaw, const double scale=1.0)
{
    std::vector<std::array<geometry_msgs::Point, 2>> result;
    double dx = scale * std::cos(yaw);
    double dy = scale * std::sin(yaw);
    double tip_x = x + dx;
    double tip_y = y + dy;
    geometry_msgs::Point p0, p1, p2, p3;
    p0.x = x;
    p0.y = y;
    p1.x = tip_x;
    p1.y = tip_y;
    result.push_back({p0, p1});

    // for the vector (dx, dy), rotate it 30 degrees and -30 degrees
    dx *= 0.8;
    dy *= 0.8;
    double angle = M_PI / 9.0;
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    double x1 = cos_angle * dx - sin_angle * dy;
    double y1 = sin_angle * dx + cos_angle * dy;
    p2.x = x + x1;
    p2.y = y + y1;
    result.push_back({p1, p2});

    angle *= -1.0;
    cos_angle = std::cos(angle);
    sin_angle = std::sin(angle);
    x1 = cos_angle * dx - sin_angle * dy;
    y1 = sin_angle * dx + cos_angle * dy;
    p3.x = x + x1;
    p3.y = y + y1;
    result.push_back({p1, p3});

    return result;
}


visualization_msgs::Marker makeLineListMarker(const std::vector<std::array<double,3>>& poses, 
                                              const std::string& frame_id)
{

    // // print the poses
    // for (size_t i = 0; i < poses.size(); i++)
    // {
    //     std::cout << "Pose " << i << ": ";
    //     for (size_t j = 0; j < poses[i].size(); j++)
    //     {
    //         std::cout << poses[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    for (size_t i = 0; i < poses.size(); i++)
    {
        double x = poses[i][0];
        double y = poses[i][1];
        double theta = poses[i][2];
        double scale = 0.35;
        auto arrow = makeArrow(x, y, theta, scale);
        for (size_t j = 0; j < arrow.size(); j++)
        {
            geometry_msgs::Point p0 = arrow[j][0];
            geometry_msgs::Point p1 = arrow[j][1];
            marker.points.push_back(p0);
            marker.points.push_back(p1);
            // std::cout << "Point " << j << ": " << p0.x << " " << p0.y << " " << p1.x << " " << p1.y << std::endl;
        }
    }

    return marker;

}



} // namespace cppad_bicycle_traj_opt

