#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch arguments
# ----------------
namespace_argument = DeclareLaunchArgument("namespace", default_value="dyno")
parameters_file_argument = DeclareLaunchArgument("parameters_file")
bag_name_argument = DeclareLaunchArgument("bag_name")
log_level_argument = DeclareLaunchArgument("log_level", default_value="WARN")

# Nodes
# -----
recorder_node = ExecuteProcess(
    cmd=[
        "ros2",
        "bag",
        "record",
        "-a",
        "-o",
        LaunchConfiguration("bag_name"),
        "--storage",
        "mcap",
    ],
    output="screen",
)

# > Native simulator interface node
native_simulator_interface_node = Node(
    namespace=[
        LaunchConfiguration("namespace"),
    ],
    name="native_simulator_interface",
    package="dyno_simulation",
    executable="dyno_simulation_native_simulator_interface_node",
    arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    parameters=[
        Path(get_package_share_path("dyno_simulation"))
        / "config/parameters/native_simulator_interface_node_defaults.yaml",
        LaunchConfiguration("parameters_file"),
    ],
    remappings=[
        # > Subscriptions
        ("controls/drive", "controls/drive"),
        # > Publishers
        ("joystick/state", "hmi/joystick/state"),
        ("joystick/feedback", "hmi/joystick/feedback"),
        ("controls/throttle", "controls/throttle"),
        ("controls/brake", "controls/brake"),
        ("controls/steering", "controls/steering"),
        ("sensors/navigation/odometry", "sensors/navigation/odometry"),
        ("sensors/navigation/imu", "sensors/navigation/imu"),
        ("sensors/steering/angle", "sensors/steering/angle"),
        ("sensors/lidar/points", "sensors/lidar/points"),
        ("sensors/camera/mono/image/raw", "sensors/camera/mono/image/raw"),
        ("sensors/camera/mono/camera_info", "sensors/camera/mono/info"),
    ],
    emulate_tty=True,
    output={
        "both": ["screen", "own_log"],
    },
    on_exit=Shutdown(),
)

# > MSDWA planner node
msdwa_planner_node = Node(
    namespace=LaunchConfiguration("namespace"),
    name="msdwa_planner",
    package="navi_planning",
    #prefix="konsole -e gdb -ex=r --args",
    executable="navi_planning_dwa_planner_node",
    arguments=[
        "--ros-args", "--log-level",
        LaunchConfiguration("log_level")
    ],
    parameters=[
        Path(
            get_package_share_path("navi_planning") /
            "config/parameters/msdwa_planner_node_defaults.yaml"
        ).as_posix(),
        LaunchConfiguration("parameters_file"),
    ],
    remappings=[
        # > Subscriptions
        ("odometry", "sensors/navigation/odometry"),
        ("goal", "navigation/target"),
        ("obstacles", "scenario/obstacles"),

        ("steering_angle", "sensors/steering/angle"),
        # > Publishers
        ("path", "planner/path"),
        ("markers", "planner/markers"),
        ("drive", "controls/drive"),
        ("path", "planner/status"),
    ],
    emulate_tty=True,
    output={
        "both": ["screen", "own_log"],
    },
    on_exit=Shutdown(),
)


def generate_launch_description():
    """Generate the launch description."""

    return LaunchDescription(
        [
            # > Launch arguments
            namespace_argument,
            log_level_argument,
            bag_name_argument,
            parameters_file_argument,
            # > Nodes
            recorder_node,
            native_simulator_interface_node,
            msdwa_planner_node,
        ]
    )


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == "__main__":
    main()
