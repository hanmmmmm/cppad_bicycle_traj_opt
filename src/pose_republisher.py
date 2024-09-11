#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class PoseRepublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_republisher', anonymous=True)

        # Create a subscriber for the /move_base_simple/goal topic
        self.subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)

        # Create a publisher for the /move_base_simple/goal topic
        self.publisher = rospy.Publisher('/goal_pose', PoseStamped, queue_size=10)

        # Store the latest pose
        self.latest_pose = PoseStamped()
        self.got_msg = False

        # Set up the rate at which to publish (e.g., 10 Hz)
        self.rate = rospy.Rate(10)

    def callback(self, msg):
        # Update the latest pose with the received message
        self.latest_pose = msg
        if not self.got_msg:
            rospy.loginfo('Received first message')
            self.got_msg = True

    def run(self):
        while not rospy.is_shutdown():
            if not self.got_msg:
                continue
            # Publish the latest pose
            self.publisher.publish(self.latest_pose)
            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pose_republisher = PoseRepublisher()
        pose_republisher.run()
    except rospy.ROSInterruptException:
        pass
