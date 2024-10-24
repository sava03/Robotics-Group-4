#!/usr/bin/env python

import rospy
from locobot_simulation.msg import LogicalImage  # Import the message type
from std_msgs.msg import Float64

class LogicalCameraSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('logical_camera_with_pan', anonymous=True)

        # Create a subscriber for the logical camera image
        self.subscriber = rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.logical_camera_callback)

        # Create publishers for pan and tilt commands
        self.pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
        self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

        # Set the publishing rate (e.g., 10 Hz)
        self.rate = rospy.Rate(10)  # 10Hz

        # Initialize pan and tilt positions
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.3  # Slight tilt upwards to get a better view
        self.delta_pan = 0.01  # Increment for panning

    def logical_camera_callback(self, data):
        for model in data.models:
            if model.type == "red_target_area":
                rospy.loginfo("Red target area detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
            elif model.type == "blue_target_area":
                rospy.loginfo("Blue target area detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
            else:
                rospy.loginfo("Other object detected: {}".format(model.type))

    def pan_camera(self):
        while not rospy.is_shutdown():
            # Publish the pan and tilt commands
            rospy.loginfo('Panning to position: {}'.format(self.pan_cmd))
            self.pan_publisher.publish(self.pan_cmd)
            self.tilt_publisher.publish(self.tilt_cmd)

            # Change pan direction when reaching limits
            if self.pan_cmd >= 1.8 or self.pan_cmd <= -1.8:
                self.delta_pan *= -1

            # Update pan position
            self.pan_cmd += self.delta_pan

            # Sleep to maintain the rate
            self.rate.sleep()

    def run(self):
        # Run both the pan camera logic and listen for camera images
        rospy.loginfo("Starting logical camera subscriber with pan control")
        self.pan_camera()


if __name__ == '__main__':
    try:
        camera_subscriber = LogicalCameraSubscriber()
        camera_subscriber.run()
    except rospy.ROSInterruptException:
        pass