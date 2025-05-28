#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from moveit.planning import MoveItPy
from moveit import PlanRequestParameters

class MoveItPoseGoalLinearNode(Node):
    def __init__(self):
        super().__init__("moveit_py")
        self.logger = self.get_logger()

        # MoveItPy setup
        self.ur_moveit = MoveItPy(node_name="moveit_py")
        self.ur_arm = self.ur_moveit.get_planning_component("ur_robot")

        # Set start state to current state
        self.ur_arm.set_start_state_to_current_state()

        # Create the pose goal
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "ur10e_base_link"
        self.pose_goal.header.frame_id = "ur10e_tool0"

        # self.pose_goal.pose.orientation.w = 1.0
        self.pose_goal.pose.position.x = 0.08
        self.tf_broadcaster = TransformBroadcaster(self)
        self.broadcast_pose_goal_tf()
        # Set goal in MoveIt
        self.ur_arm.set_goal_state(pose_stamped_msg=self.pose_goal, 
                                   pose_link="ur10e_tool0")

        plan_request_parameters : PlanRequestParameters = PlanRequestParameters(
            self.ur_moveit
        )
        plan_request_parameters.planning_pipeline = "pilz_industrial_motion_planner"
        plan_request_parameters.planner_id = "LIN"
        plan_request_parameters.max_velocity_scaling_factor = 0.05
        plan_request_parameters.max_acceleration_scaling_factor = 0.05
        plan_request_parameters.planning_time = 50.0

        # Plan to the target pose
        plan_result = self.ur_arm.plan(plan_request_parameters)
        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.ur_moveit.execute("ur_robot", robot_trajectory, blocking=True)
        else:
            self.logger.error("Planning failed")


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPoseGoalLinearNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
