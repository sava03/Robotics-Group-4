# #!/usr/bin/env python

# import rospy
# from locobot_simulation.msg import LogicalImage  # Import the message type
# from std_msgs.msg import Float64

# class LogicalCameraSubscriber:
#     def __init__(self, item_type):
#         # Initialize the ROS node
#         rospy.init_node('logical_camera_with_pan', anonymous=True)

#         # Store the item type to search for
#         self.item_type = item_type

#         # Create a subscriber for the logical camera image
#         self.subscriber = rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.logical_camera_callback)

#         # Create publishers for pan and tilt commands
#         self.pan_publisher = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=10)
#         self.tilt_publisher = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=10)

#         # Set the publishing rate (e.g., 10 Hz)
#         self.rate = rospy.Rate(10)  # 10Hz

#         # Initialize pan and tilt positions
#         self.pan_cmd = 0.0
#         self.tilt_cmd = 0.3  # Slight tilt upwards to get a better view
#         self.delta_pan = 0.01  # Increment for panning

#         # Flag to indicate if the item has been found
#         self.item_found = False

#     def logical_camera_callback(self, data):
#         for model in data.models:
#             print("model type is", model.type)
#             # Check if the detected model type matches the specified item_type
#             if model.type == self.item_type:
#                 rospy.loginfo("{} detected at position: x={}, y={}, z={}".format(
#                     self.item_type, model.pose.position.x, model.pose.position.y, model.pose.position.z))

#                 # Set item_found to True to stop the camera movement
#                 self.item_found = True
#                 break  # Stop checking other models once the target item is found

#     def pan_camera(self):
#         while not rospy.is_shutdown() and not self.item_found:
#             # Publish the pan and tilt commands
#             rospy.loginfo('Panning to position: {}'.format(self.pan_cmd))
#             self.pan_publisher.publish(self.pan_cmd)
#             self.tilt_publisher.publish(self.tilt_cmd)

#             # Change pan direction when reaching limits
#             if self.pan_cmd >= 1.8 or self.pan_cmd <= -1.8:
#                 self.delta_pan *= -1

#             # Update pan position
#             self.pan_cmd += self.delta_pan

#             # Sleep to maintain the rate
#             self.rate.sleep()

#         if self.item_found:
#             rospy.loginfo("Item found! Stopping camera movement.")

#     def run(self):
#         # Run both the pan camera logic and listen for camera images
#         rospy.loginfo("Starting logical camera subscriber with pan control")
#         self.pan_camera()


# if __name__ == '__main__':
#     try:
#         # Set the item type you're searching for
#         target_item = "red_small_ball"
#         camera_subscriber = LogicalCameraSubscriber(item_type=target_item)
#         camera_subscriber.run()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python

import rospy
from locobot_simulation.msg import LogicalImage  # Assuming the logical camera uses this message
from std_msgs.msg import Float64
from math import pi

class ArmGripperMovementClient(object):
    def __init__(self):
        # Initialize publishers for each joint
        self.joint_publishers = {
            "waist": rospy.Publisher('/locobot/waist_controller/command', Float64, queue_size=10),
            "shoulder": rospy.Publisher('/locobot/shoulder_controller/command', Float64, queue_size=10),
            "elbow": rospy.Publisher('/locobot/elbow_controller/command', Float64, queue_size=10),
            "forearm_roll": rospy.Publisher('/locobot/forearm_roll_controller/command', Float64, queue_size=10),
            "wrist_angle": rospy.Publisher('/locobot/wrist_angle_controller/command', Float64, queue_size=10),
            "wrist_rotate": rospy.Publisher('/locobot/wrist_rotate_controller/command', Float64, queue_size=10),
            "left_finger": rospy.Publisher('/locobot/left_finger_controller/command', Float64, queue_size=10),
            "right_finger": rospy.Publisher('/locobot/right_finger_controller/command', Float64, queue_size=10)
        }
        
        rospy.loginfo("ArmGripperMovementClient initialized.")

    def move_arm(self, joint_positions, duration=2.0):
        # Publish positions to each joint controller
        joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        for i, joint in enumerate(joint_names):
            rospy.loginfo(f"Moving {joint} to position {joint_positions[i]}")
            self.joint_publishers[joint].publish(Float64(joint_positions[i]))
        rospy.sleep(duration)

    def move_gripper(self, gripper_positions, duration=1.0):
        # Publish positions to both finger controllers
        rospy.loginfo(f"Moving gripper to position {gripper_positions}")
        self.joint_publishers["left_finger"].publish(Float64(gripper_positions[0]))
        self.joint_publishers["right_finger"].publish(Float64(gripper_positions[1]))
        rospy.sleep(duration)

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
        starting_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Home position
        self.move_arm(starting_position)

class LogicalCameraWithPick:
    def __init__(self):
        rospy.init_node('logical_camera_with_pick_and_place', anonymous=True)
        
        # Create the client object for arm and gripper control
        self.arm_gripper_client = ArmGripperMovementClient()
        
        # Subscriber for the logical camera
        rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.logical_camera_callback)
        
        # Gripper state
        self.gripped = False

    def logical_camera_callback(self, data):
        # Check for small red or blue objects in the camera feed
        for model in data.models:
            if model.type == "red_cube":
                rospy.loginfo("Small red cube detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
                if not self.gripped:
                    self.pick_object(model.pose.position)

            elif model.type == "blue_cube":
                rospy.loginfo("Small blue cube detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
                if not self.gripped:
                    self.pick_object(model.pose.position)
            
            elif model.type == "red_cylinder":
                rospy.loginfo("Small red cylinder detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
                if not self.gripped:
                    self.pick_object(model.pose.position)

            elif model.type == "blue_cylinder":
                rospy.loginfo("Small blue cylinder detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
                if not self.gripped:
                    self.pick_object(model.pose.position)

            elif model.type == "red_small_ball":
                rospy.loginfo("Small red cylinder detected at position: x={}, y={}, z={}".format(
                    model.pose.position.x, model.pose.position.y, model.pose.position.z))
                if not self.gripped:
                    self.pick_object(model.pose.position)

    def pick_object(self, object_position):
        # Move the arm to a predefined position based on the object's position
        rospy.loginfo("Picking object at: {}".format(object_position))
        
        # Example target position (adjust based on your setup)
        target_joint_positions = [0.5, 0.8, -0.5, pi / 2, 0.0, pi / 2]  # Adjust these values as needed
        
        # Move the arm above the object
        self.arm_gripper_client.move_to_target(target_joint_positions)
        
        # Open the gripper
        self.arm_gripper_client.open_gripper()
        
        # Lower the arm to grip position (modify based on object location)
        grip_position = [0.5, 0.6, -0.7, pi / 2, 0.0, pi / 2]  # Lower the arm for gripping
        self.arm_gripper_client.move_to_target(grip_position)
        
        # Close the gripper to pick the object
        self.arm_gripper_client.close_gripper()
        self.gripped = True
        
        # Optionally, raise the arm back to its previous height
        rospy.loginfo("Object picked, lifting...")
        self.arm_gripper_client.move_to_target(target_joint_positions)
        
        # Optionally, move the arm to a predefined drop-off location
        drop_off_position = [0.0, 0.6, -0.2, pi / 2, 0.0, pi / 2]  # Example position for drop-off
        self.arm_gripper_client.move_to_target(drop_off_position)
        
        # Open the gripper to release the object
        self.arm_gripper_client.open_gripper()
        self.gripped = False

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        # Create the logical camera subscriber and pick/place handler
        camera_pick = LogicalCameraWithPick()
        camera_pick.run()
    except rospy.ROSInterruptException:
        pass
