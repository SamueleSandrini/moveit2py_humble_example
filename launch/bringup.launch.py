import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    srdf_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'ur_robotiq.srdf')
    joint_limits_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'joint_limits.yaml')
    moveit_controllers_path = os.path.join(get_package_share_directory('ur_robotiq_bringup'), 'config', 'moveit_controllers.yaml')
    pilz_limits_path = os.path.join(get_package_share_directory('ur_robotiq_moveit_config'), 'config', 'pilz_cartesian_limits.yaml')
    robot_description_path = os.path.join(get_package_share_directory('ur_robotiq_description'), 'urdf', 'ur_robotiq.urdf.xacro') 

    moveit_config = (
        MoveItConfigsBuilder('ur_robotiq', package_name='ur_robotiq_moveit_config')
        .robot_description(file_path=robot_description_path,
                            mappings= {"fake_ur": 'true',
                                       "fake_gripper": 'true', 
                                       "ur_type": 'ur10e', 
                                       "robot_name": 'ur10e', 
                                       "tf_prefix": 'ur10e_', 
                                       "tool_device_name": '/tmp/ttyUR', 
                                       "use_tool_communication": 'false', 
                                       "tool_tcp_port": '54321', 
                                       "headless_mode": 'true',})
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .pilz_cartesian_limits(file_path=pilz_limits_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .robot_description_kinematics()
        .moveit_cpp(
            file_path=get_package_share_directory("moveit2py_humble_example")
            + "/config/motion_planning_python.yaml"
        )

        .to_moveit_configs()
    )

    example_file_cmd = DeclareLaunchArgument(
        "example_file",
        default_value="joint_goal",
        description="Python API tutorial file name",
    )
    

    moveit_py_node = Node(
        package="moveit2py_humble_example",
        executable=LaunchConfiguration('example_file'),
        name="moveit_py",
        parameters=[
            moveit_config.to_dict()
        ],
        output='screen'
    )

    

    return LaunchDescription(
        [
            example_file_cmd,
            moveit_py_node,
        ]
    )