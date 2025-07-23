#!/usr/bin/env python3

import rospy
import moveit_commander
from franka_gripper.msg import GraspActionGoal
from moveit_commander import PlanningSceneInterface

# Initialize ROS node, MoveIt Commander, and Planning Scene
rospy.init_node("pick_operation_script", anonymous=True)
moveit_commander.roscpp_initialize([])
scene = PlanningSceneInterface()

# Set up MoveIt MoveGroupCommanders
arm_group = moveit_commander.MoveGroupCommander("panda_arm")
gripper_group = moveit_commander.MoveGroupCommander("panda_hand")

# Define the joint positions for the picking motion
pick_joint_positions_1 = [-0.430, -1.713, 1.209, -2.979, 1.988, 2.728, 0.073]
pick_joint_positions_2 = [-0.749, -1.720, 1.231, -2.875, 2.251, 2.528, -0.134]
pick_joint_positions_3 = [-0.881, -1.641, 0.985, -2.749, 2.012, 2.458, -0.115]
pick_joint_positions_4 = [-1.804, -0.526, 0.706, -2.373, 1.894, 1.360, -1.104]

# Reset the planning scene
def reset_planning_scene():
    rospy.loginfo("Resetting planning scene...")
    scene.clear()
    rospy.sleep(1)  # Allow time for MoveIt to process the scene reset

# Open the gripper
def open_gripper():
    gripper_group.set_joint_value_target([0.035, 0.035])  # Values based on the open state
    gripper_group.go(wait=True)
    rospy.loginfo("Gripper opened.")

# Plan and execute a motion for given joint positions
def move_to_joint_positions(joint_positions):
    arm_group.set_joint_value_target(joint_positions)
    plan = arm_group.plan()
    
    if plan and isinstance(plan, tuple):  # Check if plan is valid and a tuple
        trajectory = plan[1]  # Extract RobotTrajectory from the tuple
        if trajectory and hasattr(trajectory, "joint_trajectory"):
            arm_group.execute(trajectory, wait=True)
            rospy.loginfo(f"Moved to joint positions: {joint_positions}")
        else:
            rospy.logerr("Invalid trajectory generated. Plan may have failed.")
    elif not isinstance(plan, tuple):  # For older versions of MoveIt
        arm_group.execute(plan, wait=True)
        rospy.loginfo(f"Moved to joint positions: {joint_positions}")
    else:
        rospy.logwarn("Failed to plan to joint positions!")

# Perform a grasp operation
def perform_grasp():
    grasp_publisher = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=10)
    rospy.sleep(1)  # Allow the publisher to initialize

    grasp_goal = GraspActionGoal()
    grasp_goal.goal.width = 0.03  # Desired grasp width
    grasp_goal.goal.speed = 0.1  # Grasping speed
    grasp_goal.goal.force = 50.0  # Grasping force
    grasp_goal.goal.epsilon.inner = 0.005  # Inner tolerance
    grasp_goal.goal.epsilon.outer = 0.005  # Outer tolerance

    grasp_publisher.publish(grasp_goal)
    rospy.sleep(2)  # Ensure the grasp action completes
    rospy.loginfo("Grasp command sent to gripper.")

# Clean up resources
def cleanup():
    moveit_commander.roscpp_shutdown()
    rospy.loginfo("MoveIt resources shut down.")

# Execute the pick operation
if __name__ == "__main__":
    try:
        rospy.loginfo("Starting pick operation...")
        reset_planning_scene()

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
        
        # Move to first place position
        rospy.loginfo("Moving to the first place position...")
        move_to_joint_positions(pick_joint_positions_3)
        
        # Move to second place position
        rospy.loginfo("Moving to the second place position...")
        move_to_joint_positions(pick_joint_positions_4)
        
        # Open the gripper
        open_gripper()

        rospy.loginfo("Pick and Place operation completed.")
    except rospy.ROSInterruptException:
        rospy.logerr("Pick and Place operation interrupted!")
    finally:
        cleanup()

