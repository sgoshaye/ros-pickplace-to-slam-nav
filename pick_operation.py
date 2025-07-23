#!/usr/bin/env python3

import rospy
import moveit_commander
from franka_gripper.msg import GraspActionGoal

# Initialize the ROS node and MoveIt Commander
rospy.init_node("pick_operation_script", anonymous=True)
moveit_commander.roscpp_initialize([])

# Set up MoveIt MoveGroupCommander
arm_group = moveit_commander.MoveGroupCommander("panda_arm")
gripper_group = moveit_commander.MoveGroupCommander("panda_hand")

# Define the joint positions for the picking motion
pick_joint_positions_1 = [-0.430, -1.713, 1.209, -2.979, 1.988, 2.728, 0.073]
pick_joint_positions_2 = [-0.749, -1.720, 1.231, -2.875, 2.251, 2.528, -0.134]

# Open the gripper
def open_gripper():
    gripper_group.set_joint_value_target([0.035, 0.035])  # Values based on open state
    gripper_group.go(wait=True)
    rospy.loginfo("Gripper opened.")

# Plan and execute a motion for given joint positions
def move_to_joint_positions(joint_positions):
    arm_group.set_joint_value_target(joint_positions)
    plan = arm_group.plan()
    if plan:
        arm_group.execute(plan, wait=True)
        rospy.loginfo(f"Moved to joint positions: {joint_positions}")
    else:
        rospy.logwarn("Failed to plan to joint positions!")

# Perform a grasp operation
def perform_grasp():
    grasp_publisher = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    grasp_goal = GraspActionGoal()
    grasp_goal.goal.width = 0.03  # Desired grasp width
    grasp_goal.goal.speed = 0.1  # Grasping speed
    grasp_goal.goal.force = 10.0  # Grasping force
    grasp_goal.goal.epsilon.inner = 0.005  # Inner tolerance
    grasp_goal.goal.epsilon.outer = 0.005  # Outer tolerance

    grasp_publisher.publish(grasp_goal)
    rospy.loginfo("Grasp command sent to gripper.")

# Execute the pick operation
if __name__ == "__main__":
    try:
        rospy.loginfo("Starting pick operation...")

        # Open the gripper
        open_gripper()

        # Move to the first pick position
        rospy.loginfo("Moving to the first pick position...")
        move_to_joint_positions(pick_joint_positions_1)

        # Move to the second pick position
        rospy.loginfo("Moving to the second pick position...")
        move_to_joint_positions(pick_joint_positions_2)

        # Perform grasp
        rospy.loginfo("Executing grasp...")
        perform_grasp()

        rospy.loginfo("Pick operation completed.")
    except rospy.ROSInterruptException:
        rospy.logerr("Pick operation interrupted!")
import rospy
import moveit_commander
from franka_gripper.msg import GraspActionGoal

# Initialize the ROS node and MoveIt Commander
rospy.init_node("pick_operation_script", anonymous=True)
moveit_commander.roscpp_initialize([])

# Set up MoveIt MoveGroupCommander
arm_group = moveit_commander.MoveGroupCommander("panda_arm")
gripper_group = moveit_commander.MoveGroupCommander("panda_hand")

# Define the joint positions for the picking motion
pick_joint_positions_1 = [-0.430, -1.713, 1.209, -2.979, 1.988, 2.728, 0.073]
pick_joint_positions_2 = [-0.749, -1.720, 1.231, -2.875, 2.251, 2.528, -0.134]

# Open the gripper
def open_gripper():
    gripper_group.set_joint_value_target([0.035, 0.035])  # Values based on open state
    gripper_group.go(wait=True)
    rospy.loginfo("Gripper opened.")

# Plan and execute a motion for given joint positions
def move_to_joint_positions(joint_positions):
    arm_group.set_joint_value_target(joint_positions)
    plan = arm_group.plan()
    if plan:
        arm_group.execute(plan, wait=True)
        rospy.loginfo(f"Moved to joint positions: {joint_positions}")
    else:
        rospy.logwarn("Failed to plan to joint positions!")

# Perform a grasp operation
def perform_grasp():
    grasp_publisher = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    grasp_goal = GraspActionGoal()
    grasp_goal.goal.width = 0.03  # Desired grasp width
    grasp_goal.goal.speed = 0.1  # Grasping speed
    grasp_goal.goal.force = 10.0  # Grasping force
    grasp_goal.goal.epsilon.inner = 0.005  # Inner tolerance
    grasp_goal.goal.epsilon.outer = 0.005  # Outer tolerance

    grasp_publisher.publish(grasp_goal)
    rospy.loginfo("Grasp command sent to gripper.")

# Execute the pick operation
if __name__ == "__main__":
    try:
        rospy.loginfo("Starting pick operation...")

        # Open the gripper
        open_gripper()

        # Move to the first pick position
        rospy.loginfo("Moving to the first pick position...")
        move_to_joint_positions(pick_joint_positions_1)

        # Move to the second pick position
        rospy.loginfo("Moving to the second pick position...")
        move_to_joint_positions(pick_joint_positions_2)

        # Perform grasp
        rospy.loginfo("Executing grasp...")
        perform_grasp()

        rospy.loginfo("Pick operation completed.")
    except rospy.ROSInterruptException:
        rospy.logerr("Pick operation interrupted!")
