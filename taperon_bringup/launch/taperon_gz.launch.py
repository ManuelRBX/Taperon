from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ---- Args ----
    world_file = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("taperon_bringup"), "worlds", "taperon_wall.sdf"
        ])
    )
    world_name = DeclareLaunchArgument("world_name", default_value="taperon_world")
    model_file = DeclareLaunchArgument(
        "model_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("taperon_description"), "urdf", "taperon.urdf"
        ])
    )
    model_name = DeclareLaunchArgument("model_name", default_value="taperon")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.5")
    R_arg = DeclareLaunchArgument("R", default_value="0.0")
    P_arg = DeclareLaunchArgument("P", default_value="0.0")
    Y_arg = DeclareLaunchArgument("Y", default_value="0.0")
        
    taperon_xacro = PathJoinSubstitution([
        FindPackageShare("taperon_description"),
        "urdf",
        "taperon.urdf.xacro",   
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            taperon_xacro
        ]),
        value_type=str,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="taperon",  
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ---- Software rendering env (your setup) ----
    env_soft = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    env_llvmpipe = SetEnvironmentVariable("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")

    # ---- IMPORTANT: plugin search path ----
    env_ign_sys_plugins = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )
    env_gz_sys_plugins = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )

    # ---- Gazebo (Ignition) ----
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={"gz_args": LaunchConfiguration("world_file")}.items()
    )

    # ---- Spawn command ----
    spawn_cmd = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-world", LaunchConfiguration("world_name"),
            "-name", LaunchConfiguration("model_name"),
            "-file", LaunchConfiguration("model_file"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-R", LaunchConfiguration("R"),
            "-P", LaunchConfiguration("P"),
            "-Y", LaunchConfiguration("Y"),
        ],
        output="screen"
    )

    spawn_after_gz = TimerAction(period=2.0, actions=[spawn_cmd])

    cfg_path = PathJoinSubstitution([
        FindPackageShare("taperon_bringup"), "config", "diff_drive.yaml"
    ])

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-type", "joint_state_broadcaster/JointStateBroadcaster",
        ],
        output="screen",
    )

    dd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", cfg_path],
        output="screen"
    )

    spawn_controllers = TimerAction(period=3.0, actions=[jsb, dd])

    return LaunchDescription([
        env_soft, env_llvmpipe,
        env_ign_sys_plugins, env_gz_sys_plugins,     
        world_file, world_name, model_file, model_name,
        x_arg, y_arg, z_arg, R_arg, P_arg, Y_arg,
        robot_state_pub,
        gz,
        spawn_after_gz,
        spawn_controllers
    ])
