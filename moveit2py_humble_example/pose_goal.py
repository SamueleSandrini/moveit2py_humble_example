import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from moveit.planning import MoveItPy

class MoveItPoseGoalNode(Node):
    def __init__(self):
        super().__init__("moveit_py_pose_goal_broadcaster")
        self.logger = self.get_logger()

        # MoveItPy setup
        self.ur_moveit = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.ur_moveit.get_planning_component("ur_robot")

        # Set start state to current state
        self.panda_arm.set_start_state_to_current_state()

        # Create the pose goal
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "ur10e_base_link"
        self.pose_goal.pose.orientation.w = 1.0
        self.pose_goal.pose.position.x = -0.279
        self.pose_goal.pose.position.y = -0.16
        self.pose_goal.pose.position.z = 0.8

        # Set goal in MoveIt
        self.panda_arm.set_goal_state(pose_stamped_msg=self.pose_goal, 
                                      pose_link="ur10e_tool0")

        # Plan to the target pose
        plan_result = self.panda_arm.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.ur_moveit.execute("ur_robot", robot_trajectory, blocking=True)
        else:
            self.logger.error("Planning failed")

        # Setup tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.broadcast_pose_goal_tf()

    def broadcast_pose_goal_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.pose_goal.header.frame_id
        t.child_frame_id = "pose_goal_frame" 

        t.transform.translation.x = self.pose_goal.pose.position.x
        t.transform.translation.y = self.pose_goal.pose.position.y
        t.transform.translation.z = self.pose_goal.pose.position.z

        t.transform.rotation = self.pose_goal.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPoseGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
