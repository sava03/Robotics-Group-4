#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
import sys
import signal


class AutonomousRobot():
    def __init__(self, goal_x, goal_y, goal_theta, topic="/locobot/mobile_base/commands/velocity"):
        self.topic = topic
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Initialize ROS node and publisher
        rospy.init_node('autonomous_robot')
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_callback(self, msg):
        # Get the current position from the odometry data
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        print("stats:", self.current_x, self.current_y, orientation_q)
        _, _, self.current_theta = self.euler_from_quaternion(orientation_q)

    def euler_from_quaternion(self, orientation_q):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        import tf
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler  # returns roll, pitch, yaw

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def angle_to_goal(self):
        return math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

    def run(self):
        # Set signal handler for SIGINT (CTRL+C)
        signal.signal(signal.SIGINT, self.cleanup)

        while not rospy.is_shutdown():
            print("aaaa")
            # Calculate distance and angle to goal
            distance = self.distance_to_goal()
            angle_to_goal = self.angle_to_goal()

            # Proportional control for linear and angular velocity
            linear_speed = min(0.5 * distance, 0.3)  # max speed = 0.3
            angular_speed = 2.0 * (angle_to_goal - self.current_theta)  # proportional controller

            # If the robot is close to the goal, stop
            if distance < 0.05:
                linear_speed = 0.0
                angular_speed = 0.0

            # Create Twist message
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            # Publish Twist message
            self.pub.publish(twist)

            # Debugging output
            rospy.loginfo(f"Distance to goal: {distance:.2f}, Linear speed: {linear_speed:.2f}, Angular speed: {angular_speed:.2f}")

            self.rate.sleep()

    def cleanup(self, signal, frame):
        # Stop the robot when the program quits
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        sys.exit(0)


if __name__ == '__main__':
    # Goal position and orientation (in radians)
    goal_x = 1.0  # desired x position
    goal_y = 1.0  # desired y position
    goal_theta = 0.0  # desired orientation (theta)

    if len(sys.argv) > 3:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_theta = float(sys.argv[3])

    try:
        robot = AutonomousRobot(goal_x, goal_y, goal_theta)
        robot.run()
    except rospy.ROSInterruptException:
        pass
