#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class PublisherNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('publisher_node', anonymous=True)

        # Create publishers
        self.pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
        self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

        # Set the publishing rate
        self.rate = rospy.Rate(100)  # 100 Hz

        # Set initial commands for pan & tilt
        self.pan_cmd = 0.0  # Static pan position
        self.tilt_cmd = 0.3  # Static tilt position

        # Publish the initial positions
        self.pan_publisher.publish(self.pan_cmd)
        self.tilt_publisher.publish(self.tilt_cmd)

    def publish_topics(self):
        while not rospy.is_shutdown():
            # Create messages
            topic1_msg = self.pan_cmd
            topic2_msg = self.tilt_cmd

            # Publish messages
            rospy.loginfo('Publishing command: pan={}, tilt={}'.format(self.pan_cmd, self.tilt_cmd))
            self.pan_publisher.publish(topic1_msg)
            self.tilt_publisher.publish(topic2_msg)

            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node = PublisherNode()
        publisher_node.publish_topics()
    except rospy.ROSInterruptException:
        pass
