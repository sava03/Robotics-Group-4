#!/usr/bin/env python
"""
In this file, a camera class and a robot class are created. The program subscirbes to the 
logical camera topic to obtain information about the objects it observes.
(in a previous iteration the camera looked around first)
Then, the robot starts moving forward and controls the arm joints to enter the grip
position for small objects. Depending on what the camera sees, the shoulder joint
is rotated such that these objects are in reach. Then, the gripper closes and (should)
retract with the small object in hand and return to starting pos.

"""
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import sys
import signal
import math
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from locobot_simulation.msg import LogicalImage
from std_msgs.msg import Float64

class LogicalCamera:
    def __init__(self, robot):
        self.robot = robot
        rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.logical_camera_callback)
        self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)
        self.tilt_cmd = 0.3  # Static tilt position
        self.tilt_publisher.publish(self.tilt_cmd)

        self.gripped = False
        self.detected_object = None  # Store detected object type

    def logical_camera_callback(self, data):
        for model in data.models:
         #["red_cube", "blue_cube", "red_cylinder", "blue_cylinder", "red_small_ball"] Add more late
            if model.type in ["blue_cube", "blue_small_ball", "red_small_ball"]:
                rospy.loginfo(f"{model.type} detected at position: x={model.pose.position.x}, y={model.pose.position.y}, z={model.pose.position.z}")
                self.detected_object = model.type

class CompleteRobot:
    def __init__(self, topic="/locobot/mobile_base/commands/velocity"):
        rospy.init_node('complete_robot', anonymous=True)
        self.topic = topic
        self.start_position = None
        self.current_position = None
        self.current_orientation = None

        # Gripper setup
        self.arm_client = actionlib.SimpleActionClient('/locobot/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller...")
        self.arm_client.wait_for_server()

        self.gripper_client = actionlib.SimpleActionClient('/locobot/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()

        self.starting_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Initialize ROS publisher and subscriber
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/locobot/mobile_base/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10Hz

        signal.signal(signal.SIGINT, self.cleanup)

        # Initialize LogicalCamera
        self.logical_camera = LogicalCamera(self)

    def odom_callback(self, msg):
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if not self.start_position:
            self.start_position = self.current_position[:]
            rospy.loginfo(f"Saved start position: x={self.start_position[0]:.2f}, y={self.start_position[1]:.2f}")

        orientation_quat = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.current_orientation = yaw

    def move(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        for _ in range(int(duration * 10)):  # 10Hz rate
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def back_to_start(self):
        if self.start_position is None:
            rospy.logwarn("Start position not set, cannot return to start.")
            return

        rospy.loginfo("Moving back to start position...")
        while self.distance_to_start() > 0.1:
            dx = self.start_position[0] - self.current_position[0]
            dy = self.start_position[1] - self.current_position[1]

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_orientation
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            twist = Twist()
            twist.linear.x = 0.2  # Move forward slowly

            if abs(angle_diff) > 0.1:  # 0.1 radian threshold
                twist.angular.z = 0.2 if angle_diff > 0 else -0.2  # Turn left or right
            else:
                twist.angular.z = 0.0  # Go straight

            self.pub.publish(twist)
            self.rate.sleep()

        self.stop_robot()
        rospy.loginfo("Returned to start position.")

    def distance_to_start(self):
        if not self.current_position or not self.start_position:
            return float('inf')
        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        return (dx**2 + dy**2)**0.5

    def move_arm(self, joint_positions, duration=2.0):
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)

        arm_goal.trajectory.points.append(point)
        arm_goal.goal_time_tolerance = rospy.Duration(0.5)

        rospy.loginfo("Sending arm movement goal...")
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

    def move_gripper(self, gripper_positions, duration=1.0):
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory.joint_names = ["left_finger", "right_finger"]

        point = JointTrajectoryPoint()
        point.positions = gripper_positions
        point.time_from_start = rospy.Duration(duration)

        gripper_goal.trajectory.points.append(point)

        rospy.loginfo("Sending gripper movement goal...")
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()

    def open_gripper(self):
        rospy.loginfo("Opening gripper...")
        open_position = [1.0, -1.0]  # Maximum opening
        self.move_gripper(open_position)

    def close_gripper(self):
        rospy.loginfo("Closing gripper...")
        close_position = [-0.1, 0.1]  # Fully closed
        self.move_gripper(close_position)

    def return_to_start(self):
        rospy.loginfo("Returning the arm to the starting position...")
        self.move_arm(self.starting_position)

    def get_target_joint_positions(self, object_type):
        base_joint_positions = [0.67, 0.19, 0.0, 0.7, -0.2]  # Standard position without the first joint

        if object_type == 'blue_cube':
            first_joint_position = -0.21
        elif object_type == 'blue_small_ball':
            first_joint_position = 0.30  
        elif object_type == 'red_small_ball':
            first_joint_position = 0.4  # Example value for a red small ball
        else:
            first_joint_position = -0.21  # Default value (blue cube)

        return [first_joint_position] + base_joint_positions

    def scan_for_objects(self):
        rospy.loginfo("Scanning for objects...")
        # Wait for a short duration to ensure the camera can capture objects
        rospy.sleep(2.0)  # Adjust duration as needed

    def run(self):
        rospy.loginfo("Starting complete robot operation.")

        # Step 1: Scan for objects
        self.scan_for_objects()
        
        # Step 2: Move forward
        rospy.loginfo("Moving forward...")
        move_lin = 0.1  # initial forward speed
        move_ang = 0
        move_dur = 1.7  # duration
        self.move(move_lin, move_ang, move_dur)  # Move forward for the specified duration

        # Step 3: Open gripper
        self.open_gripper()
        rospy.sleep(0.2)

        # Step 4: Move arm based on detected object type
        target_joint_positions = self.get_target_joint_positions(self.logical_camera.detected_object)
        self.move_arm(target_joint_positions)
        rospy.sleep(0.2)
        self.close_gripper()

        # Step 5: Return arm to start position
        self.return_to_start()  

        # Move back to the start position
        rospy.loginfo("Moving back...")
        self.move(-move_lin, move_ang, move_dur)  # Move backward

    def cleanup(self, signum, frame):
        rospy.loginfo("Shutting down robot...")
        self.stop_robot()
        sys.exit(0)

if __name__ == '__main__':
    try:
        robot = CompleteRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass

