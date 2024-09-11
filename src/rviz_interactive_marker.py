#!/usr/bin/env python3


import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from std_msgs.msg import Float64MultiArray


server = None
menu_handler = MenuHandler()
br = None
counter = 0


def processFeedback( feedback ):
    # s = "Feedback from marker '" + feedback.marker_name
    # s += "' / control '" + feedback.control_name + "'"

    # mp = ""
    # if feedback.mouse_point_valid:
    #     mp = " at " + str(feedback.mouse_point.x)
    #     mp += ", " + str(feedback.mouse_point.y)
    #     mp += ", " + str(feedback.mouse_point.z)
    #     mp += " in frame " + feedback.header.frame_id

    # if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    #     rospy.loginfo( s + ": button click" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
    #     rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
    #     rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #     rospy.loginfo( s + ": mouse down" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    #     rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def makeShape():
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.02
    marker.color.r = 0.9
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 1.0
    return marker



def makeChessPieceMarker(position, name):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 0.2

    int_marker.name = str(name)
    int_marker.description = ""

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append( makeShape() )
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

    # print marker pose
    print(int_marker.pose)




if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    
    server = InteractiveMarkerServer("basic_controls")

    makeChessPieceMarker( Point( 3, 0, 0), "C1" )
    makeChessPieceMarker( Point( 5, 1, 0), "C2" )    
    
    server.applyChanges()

    rospy.spin()
