#ifndef POSE_MARKER_MAKER_H
#define POSE_MARKER_MAKER_H


#include <iostream>
#include "visualization_msgs/Marker.h"

namespace cppad_bicycle_traj_opt
{


/**
 * @brief For a list of 2D poses, create a visualization_msgs::Marker.
 * The marker is a line list, consisting of a series of line segments.
 * Each pose will be an arrow, with the arrow head pointing in the direction of the pose.
 * 
 * 
 */
visualization_msgs::Marker makeLineListMarker(const std::vector<std::array<double,3>>& poses, 
                                              const std::string& frame_id);

}

#endif // POSE_MARKER_MAKER_H