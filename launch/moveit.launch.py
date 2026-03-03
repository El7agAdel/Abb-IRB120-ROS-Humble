import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = os.path.join(get_package_share_directory("IRB120"), "worlds", "IRB120_green_cube.sdf")
                

    irb120_share = get_package_share_directory("IRB120")
    moveit_config_dir = os.path.join(irb120_share, "moveit_config")
    controllers_file = os.path.join(get_package_share_directory("IRB120"), "config/IRB120_controller.yaml")
    urdf_xacro = os.path.join(moveit_config_dir, "IRB120.urdf.xacro")
    srdf_path = os.path.join(moveit_config_dir, "IRB120.srdf")
    kinematics_path = os.path.join(moveit_config_dir, "kinematics.yaml")
    joint_limits_path = os.path.join(moveit_config_dir, "joint_limits.yaml")
    moveit_controllers_path = os.path.join(moveit_config_dir, "moveit_controllers.yaml")
    rviz_config = os.path.join(moveit_config_dir, "moveit.rviz")

    robot_description = {"robot_description": xacro.process_file(urdf_xacro).toxml()}
    robot_description_semantic = {
        "robot_description_semantic": load_file(srdf_path)
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(kinematics_path)
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(joint_limits_path)
    }
    trajectory_execution = load_yaml(moveit_controllers_path)

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }

    planning_scene_monitor = {
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

     # ---- Gazebo Fortress (ros_gz_sim)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # ---- Publish /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": use_sim_time, "robot_description": xacro.process_file(urdf_xacro).toxml()}],
        output='screen'
    )

    # ---- Spawn robot (from /robot_description)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
        '-name', 'irb120',
        '-x', '0.0',     # meters
        '-y', '0.0',
        '-z', '0.0',
        '-R', '0.0',     # roll (rad)
        '-P', '0.0',     # pitch
        '-Y', '0.0',    # yaw
        '-topic', 'robot_description'
                    ],
        output='screen'
    )

    # ---- Controllers
    arm_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file
        ],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file
        ],
        output='screen'
    )

    # ---- Optional: bridge /clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )



    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            planning_scene_monitor,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
    )

    delayed_moveit = TimerAction(period=2.5, actions=[move_group, rviz])


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            gazebo_launch,
            robot_state_publisher,
            spawn,
            arm_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            clock_bridge,
            delayed_moveit,
        ]
    )
