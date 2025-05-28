import rclpy
from rclpy.logging import get_logger

# RobotState is used to set joint values
from moveit.core.robot_state import RobotState

# MoveIt python library
from moveit.planning import (
    MoveItPy,
)
import os

def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # Instantiate MoveItPy and get planning component
    ur_moveit = MoveItPy(node_name="moveit_py")

    ur_arm = ur_moveit.get_planning_component("ur_robot")
    
    # Create a robot state to set the joint values
    robot_model = ur_moveit.get_robot_model()
    robot_state = RobotState(robot_model)
    
    # Set plan start state to current state
    ur_arm.set_start_state_to_current_state()

    # Set target joint values
    joint_values = {
        "ur10e_shoulder_pan_joint": 1.5,
        "ur10e_shoulder_lift_joint": -4.0,
        "ur10e_elbow_joint": 2.4,
        "ur10e_wrist_1_joint": -1.5,
        "ur10e_wrist_2_joint": 1.5,
        "ur10e_wrist_3_joint": 0.0,
    }

    robot_state.joint_positions = joint_values

    # # Set the joint values as the goal state
    ur_arm.set_goal_state(robot_state=robot_state)

    # # Create a plan to the target pose
    plan_result = ur_arm.plan()

    # # If the plan is successful, get the trajectory and execute the plan
    if plan_result:
        robot_trajectory = plan_result.trajectory
        ur_moveit.execute("ur_robot",robot_trajectory, blocking=True)
    else:
        logger.error("Planning failed")

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()