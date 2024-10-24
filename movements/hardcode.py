#!/usr/bin/env python
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
from locobot_simulation.msg import LogicalImage  # Assuming the logical camera uses this message


class LogicalCameraWithPick:
    def __init__(self, robot):
        self.robot = robot
        rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.logical_camera_callback)
        self.gripped = False

    def logical_camera_callback(self, data):
        for model in data.models:
            if model.type in ["red_cube", "blue_cube", "red_cylinder", "blue_cylinder", "red_small_ball"]:
                rospy.loginfo(f"{model.type} detected at position: x={model.pose.position.x}, y={model.pose.position.y}, z={model.pose.position.z}")
                if not self.gripped:
                    self.pick_object(model.pose.position, model.type)

    def pick_object(self, position, object_type):
        rospy.loginfo(f"Picking up {object_type} at position: {position}")
        # Move the arm to the object
        target_joint_positions = self.robot.get_target_joint_positions(object_type)
        self.robot.move_arm(target_joint_positions)
        
        # Close gripper
        self.robot.close_gripper()
        self.gripped = True
        rospy.sleep(0.5)

class CompleteRobot():
    def __init__(self, topic="/locobot/mobile_base/commands/velocity"):
        rospy.init_node('complete_robot', anonymous=True)  # Initialize the node first

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


    def odom_callback(self, msg):
        """Callback for updating the current position and orientation."""
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]
        if not self.start_position:
            self.start_position = self.current_position[:]
            rospy.loginfo("Saved start position: x={:.2f}, y={:.2f}".format(self.start_position[0], self.start_position[1]))

        orientation_quat = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        )
        self.current_orientation = yaw

        # rospy.loginfo("Current position: x={:.2f}, y={:.2f}, yaw={:.2f}".format(
        #     self.current_position[0], self.current_position[1], self.current_orientation))

    def move(self, linear, angular, duration):
        """Move the robot with given linear and angular velocities for a duration."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        for _ in range(int(duration * 10)):  # 10Hz rate
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop_robot()

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
        """Calculate the distance between the current and start positions."""
        if not self.current_position or not self.start_position:
            return float('inf')
        dx = self.current_position[0] - self.start_position[0]
        dy = self.current_position[1] - self.start_position[1]
        return (dx**2 + dy**2)**0.5

    def move_arm(self, joint_positions, duration=2.0):
        """Move the robot arm to the specified joint positions."""
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
        """Move the gripper to the specified positions."""
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

    def get_target_joint_positions(object_type):
        # Arm movement ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        # for small objects (bend over and angle wrist)
        base_joint_positions = [0.67, 0.19, 0.0, 0.7, -0.2]  # Standard pos without the first joint
        
        # VV add the other types later
        if object_type == 'blue_small_ball':
            first_joint_position = 0.30  
        elif object_type == 'blue_cube':
            first_joint_position = -0.21
        else:
            first_joint_position = 0.0  # Default value
        
        return [first_joint_position] + base_joint_positions


    def run(self):
    # Gameplan: Use forward kinematics to get end-effector pos. 
        rospy.loginfo("Starting complete robot operation.")
        rospy.loginfo("Moving forward...")
        move_lin = 0.1 # initially along x-axis
        move_ang = 0
        move_dur = 1.7 # <- duration should also depend on object type
        self.move(move_lin, move_ang, move_dur)  # Move forward for 2 seconds

        self.open_gripper()
        # Arm movement ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        self.move_arm(self.get_target_joint_positions('blue_cube'))
        rospy.sleep(1.0)
        # Close the gripper
        self.close_gripper()
        rospy.sleep(0.5)
        self.return_to_start()  # Return arm to start position

        # Move back to the start position
        rospy.loginfo("moving back xd")
        self.move(-move_lin, move_ang, move_dur)
        self.cleanup()

    def cleanup(self, signum=None, frame=None):
        """Cleanup actions on shutdown."""
        rospy.loginfo("Shutting down robot...")
        self.stop_robot()
        sys.exit(0)

if __name__ == "__main__":
    robot = CompleteRobot()
    robot.run()
    print("finsihed")
