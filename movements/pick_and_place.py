#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class ArmGripperMovementClient(object):
    def __init__(self):
        # Initialize action client for arm controller
        self.arm_client = actionlib.SimpleActionClient('/locobot/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller...")
        self.arm_client.wait_for_server()

        # Initialize action client for gripper controller
        self.gripper_client = actionlib.SimpleActionClient('/locobot/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()

        # Define the starting joint positions (home position)
        self.starting_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def move_arm(self, joint_positions, duration=2.0):
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]

        # Create a trajectory point for the arm
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory
        arm_goal.trajectory.points.append(point)
        arm_goal.goal_time_tolerance = rospy.Duration(0.5)

        # Send the goal to the arm action server
        rospy.loginfo("Sending arm movement goal...")
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

    def move_gripper(self, gripper_positions, duration=1.0):
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory.joint_names = ["left_finger", "right_finger"]

        # Create a trajectory point for the gripper
        point = JointTrajectoryPoint()
        point.positions = gripper_positions  # [open or close position for both fingers]
        point.time_from_start = rospy.Duration(duration)

        # Add the point to the trajectory
        gripper_goal.trajectory.points.append(point)

        # Send the goal to the gripper action server
        rospy.loginfo("Sending gripper movement goal...")
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()

    def open_gripper(self):
        rospy.loginfo("Opening gripper...")
        open_position = [0.04, 0.04]  # Maximum opening
        self.move_gripper(open_position, duration=1.0)

    def close_gripper(self):
        rospy.loginfo("Closing gripper...")
        close_position = [0.0, 0.0]  # Fully closed
        self.move_gripper(close_position, duration=1.0)

    def move_to_target(self, target_joint_positions):
        rospy.loginfo("Moving the arm to the target position...")
        self.move_arm(target_joint_positions)

    def return_to_start(self):
        rospy.loginfo("Returning the arm to the starting position...")
        self.move_arm(self.starting_position)

if __name__ == "__main__":
    print("Arm movement and gripping")
    rospy.init_node("arm_gripper_movement")

    # Ensure sim time is working
    while not rospy.Time.now():
        pass

    # Initialize the movement client
    client = ArmGripperMovementClient()

    # Target joint positions (example)
    target_joint_positions = [1.0, 0.5, -0.3, 1.2, -0.1, 0.5]

    # Move the arm to the target position
    client.move_to_target(target_joint_positions)

    # Open the gripper
    client.open_gripper()

    # Pause for a moment
    rospy.sleep(2.0)

    # Close the gripper
    client.close_gripper()

    # Return the arm to the starting position
    client.return_to_start()

    rospy.loginfo("Finished")
