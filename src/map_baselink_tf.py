#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf2_ros

def publish_transform():
    rospy.init_node('static_transform_publisher_node', anonymous=True)

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Define the translation and rotation (identity pose means no translation or rotation)
        translation = (0.0, 0.0, 0.0)  # No translation (identical pose)
        rotation = (0.0, 0.0, 0.0, 1.0)  # No rotation (identity quaternion)

        # Publish the transform between map and base_link
        br.sendTransform(translation,
                         rotation,
                         rospy.Time.now(),
                         "base_link",  # Child frame
                         "map")        # Parent frame

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
