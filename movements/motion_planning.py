import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('motion_planning_python', anonymous=True)

# Initialize RobotCommander
robot = moveit_commander.RobotCommander(robot_description='/locobot/robot_description', ns="locobot")

# Initialize PlanningSceneInterface
# scene = moveit_commander.PlanningSceneInterface(ns="locobot")

# Initialize MoveGroupCommander with the interbotix_arm planning group
group = moveit_commander.MoveGroupCommander("interbotix_arm", robot_description='/locobot/robot_description', ns="locobot")

# Publisher for displaying planned paths
display_trajectory_publisher = rospy.Publisher('/locobot/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

# Set the target pose
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.9
pose_target.position.y = 0.8
pose_target.position.z = 1.1
group.set_pose_target(pose_target)

# Plan the motion
plan1 = group.plan()

# Display the planned trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory)

# Execute the motion
group.execute(plan1, wait=True)

# Shutdown moveit_commander
moveit_commander.roscpp_shutdown()