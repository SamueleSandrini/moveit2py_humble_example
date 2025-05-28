import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject

def add_single_pick_object(planning_scene_monitor):
    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "ur10e_base_link"
        collision_object.id = "pick_box"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        box_pose = Pose()
        box_pose.position.x = 0.4
        box_pose.position.y = 0.0
        box_pose.position.z = 0.75  # ad esempio su un tavolo

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()

def attach_existing_object(planning_scene_monitor):
    with planning_scene_monitor.read_write() as scene:
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "ur10e_tool0"
        attached_object.object.id = "pick_box"
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = ["ur10e_tool0"]

        scene.process_attached_collision_object(attached_object)
        scene.current_state.update()

def attach_collision_object(planning_scene_monitor):
    with planning_scene_monitor.read_write() as scene:
        # Create a collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = "ur10e_tool0"
        collision_object.id = "attached_box"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        box_pose = Pose()
        box_pose.position.x = 0.0
        box_pose.position.y = 0.0
        box_pose.position.z = 0.25

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        # AttachedCollisionObject
        attached_object = AttachedCollisionObject()
        attached_object.link_name = "ur10e_tool0"
        attached_object.object = collision_object
        attached_object.touch_links = ["ur10e_tool0"]
        attached_object.object.operation = CollisionObject.ADD

        # Apply
        scene.process_attached_collision_object(attached_object)

        scene.current_state.update()


def add_collision_objects(planning_scene_monitor):
    """Helper function that adds collision objects to the planning scene."""
    object_positions = [
        (0.15, 0.1, 0.1),
        (0.25, 0.0, 1.0),
        (-0.25, -0.3, 0.8),
        (0.25, 0.3, 0.75),
    ]
    object_dimensions = [
        (0.1, 0.4, 0.1),
        (0.1, 0.4, 0.1),
        (0.2, 0.2, 0.2),
        (0.15, 0.15, 0.15),
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "ur10e_tool0"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated


class MoveItPoseGoalNode(Node):
    def __init__(self):
        super().__init__("moveit_py_with_objects")
        self.logger = self.get_logger()

        # MoveItPy setup
        loaded = False
        while not(loaded):
            try:
                self.ur_moveit = MoveItPy(node_name="moveit_py")
                loaded = True
            except Exception as ex:
                self.get_logger().info(f"Exception: {ex}")

        self.ur_arm = self.ur_moveit.get_planning_component("ur_robot")
        planning_scene_monitor = self.ur_moveit.get_planning_scene_monitor()
        self.logger.info("MoveItPy instance created")

        ###################################################################
        # Plan with collision objects
        ###################################################################

        # attach_collision_object(planning_scene_monitor)
        add_single_pick_object(planning_scene_monitor)

        self.logger.info("Attached collision onject")

    #     # Set start state to current state
        self.ur_arm.set_start_state_to_current_state()

    #     # Create the pose goal
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "ur10e_base_link"
        self.pose_goal.pose.orientation.w = 1.0
        self.pose_goal.pose.position.x = -0.279
        self.pose_goal.pose.position.y = -0.16
        self.pose_goal.pose.position.z = 0.8

        # Set goal in MoveIt
        self.ur_arm.set_goal_state(pose_stamped_msg=self.pose_goal, pose_link="ur10e_tool0")

        # Plan to the target pose
        plan_result = self.ur_arm.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory

            self.ur_moveit.execute("ur_robot", robot_trajectory, blocking=True)
            self.logger.error("Movement Executed")
        else:
            self.logger.error("Planning failed")
        attach_existing_object(planning_scene_monitor)
        self.ur_arm.set_start_state_to_current_state()

        #     # Create the pose goal
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "ur10e_base_link"
        self.pose_goal.pose.orientation.w = 1.0
        self.pose_goal.pose.position.x = -0.279
        self.pose_goal.pose.position.y = -0.16
        self.pose_goal.pose.position.z = 0.9

        # Set goal in MoveIt
        self.ur_arm.set_goal_state(pose_stamped_msg=self.pose_goal, 
                                   pose_link="ur10e_tool0")

        # Plan to the target pose
        plan_result = self.ur_arm.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.ur_moveit.execute("ur_robot", robot_trajectory, blocking=True)
        else:
            self.logger.error("Planning failed")
        self.ur_arm.set_start_state_to_current_state()



def main(args=None):
    rclpy.init(args=args)
    node = MoveItPoseGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
