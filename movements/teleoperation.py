"""
This code currently works with moving forward and going back to start, butttt
when the robot gets stuck(which it does. often.) the program is recognizing the stuck state
with the roswarn, but it doesnt finish correctly yet. :_:
"""


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import sys
import signal
import math
import time

class TeleopRobot():
    def __init__(self, topic="/locobot/mobile_base/commands/velocity"):
        self.topic = topic
        self.start_position = None
        self.current_position = None
        self.current_orientation = None

        # Variables to detect if stuck
        self.last_position = None
        self.stuck_start_time = None
        self.stuck_threshold = 1.0  # Time threshold in seconds to consider stuck
        self.position_stopped_threshold = 0.1  # Position change threshold to consider moving
        self.is_stuck = False  # Flag to indicate if the robot is stuck

        # Initialize ROS node and publisher
        rospy.init_node('teleop', anonymous=True)
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/locobot/mobile_base/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10Hz

        # Set signal handler for SIGINT (CTRL+C)
        signal.signal(signal.SIGINT, self.cleanup)

    def odom_callback(self, msg):
        """Callback for updating the current position and orientation."""
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        if not self.start_position:
            self.start_position = self.current_position[:]
            rospy.loginfo("Saved start position: x={:.2f}, y={:.2f}".format(self.start_position[0], self.start_position[1]))

        # Extract yaw from quaternion
        orientation_quat = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        )
        self.current_orientation = yaw
        
        # Log current position and orientation
        rospy.loginfo("Current position: x={:.2f}, y={:.2f}, yaw={:.2f}".format(
            self.current_position[0], self.current_position[1], self.current_orientation))

        self.check_if_stuck()

    def check_if_stuck(self):
        """Check if the robot is stuck by monitoring position changes."""
        if self.last_position is None:
            self.last_position = self.current_position[:]
            return

        # Calculate the distance moved since the last position update
        distance_moved = math.sqrt((self.current_position[0] - self.last_position[0])**2 +
                                    (self.current_position[1] - self.last_position[1])**2)

        # If the robot's velocity is non-zero and it's not moving
        if distance_moved < self.position_stopped_threshold:
            if self.stuck_start_time is None:
                self.stuck_start_time = time.time()
            elif time.time() - self.stuck_start_time >= self.stuck_threshold:
                rospy.logwarn("Robot is stuck! Stopping...")
                self.stop_robot()  # Stop the robot
                self.is_stuck = True  # Set the stuck flag
                return
        else:
            # Reset stuck timer if the robot is moving
            self.stuck_start_time = None
            self.is_stuck = False  # Reset the stuck flag

        self.last_position = self.current_position[:]

    def move(self, linear, angular, duration):
        """Move the robot with given linear and angular velocities for a duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        for _ in range(int(duration * 10)):  # 10Hz rate
            self.pub.publish(twist)
            self.rate.sleep()

    def stop_robot(self):
        """Stop the robot's movement."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def back_to_start(self):
        """Move the robot back to the original starting position."""
        if self.start_position is None:
            rospy.logwarn("Start position not set, cannot return to start.")
            return

        rospy.loginfo("Moving back to start position...")
        while self.distance_to_start() > 0.1:
            # Calculate the difference in position
            dx = self.start_position[0] - self.current_position[0]
            dy = self.start_position[1] - self.current_position[1]

            # Calculate the desired angle to turn towards the start position
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_orientation
            
            # Normalize the angle to the range [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            # Set the linear speed
            twist = Twist()
            twist.linear.x = 0.2  # Move forward slowly

            # If we're not facing the start position, turn towards it
            if abs(angle_diff) > 0.1:  # 0.1 radian threshold
                twist.angular.z = 0.2 if angle_diff > 0 else -0.2  # Turn left or right
            else:
                twist.angular.z = 0.0  # Go straight

            self.pub.publish(twist)
            self.rate.sleep()

        # Stop the robot after reaching the start position
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        rospy.loginfo("Returned to start position.")

    def distance_to_start(self):
        """Calculate the distance between the current and start positions."""
        if not self.current_position or not self.start_position:
            return float('inf')
        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        return (dx**2 + dy**2)**0.5

    def run(self):
        rospy.loginfo("Starting teleoperation.")

        # Move to a new position
        rospy.loginfo("Moving forward...")
        self.move(0.3, 0.0, 2)  # Move forward for 2 seconds

        # Check for stuck condition after the movement
        if self.is_stuck:
            rospy.loginfo("Robot has stopped due to being stuck. Attempting recovery...")
            self.stop_robot()  # Ensure the robot is stopped
            time.sleep(1)  # Wait for a brief moment to reset

            # Attempt to recover by moving back and forth
            self.move(-0.2, 0.0, 1)  # Move backward for 1 second
            self.move(0.2, 0.0, 1)   # Move forward for 1 second

            # Reset stuck condition
            self.stuck_start_time = None

            # Recheck the stuck condition
            self.check_if_stuck()
            if self.is_stuck:
                rospy.logwarn("Robot is still stuck after recovery attempt.")
                self.cleanup()  # Call cleanup to exit

        # Move back to the start position
        self.back_to_start()

        rospy.loginfo("Teleoperation finished.")
        self.cleanup()  # Call cleanup to ensure proper exit

    def cleanup(self, signum=None, frame=None):
        """Stop the robot and exit."""
        self.stop_robot()
        rospy.signal_shutdown("Shutting down teleop node.")
        sys.exit(0)

if __name__ == '__main__':
    topic = "/locobot/mobile_base/commands/velocity"
    try:
        teleop = TeleopRobot(topic)
        teleop.run()
    except rospy.ROSInterruptException:
        pass
