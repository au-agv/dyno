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
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch arguments
# ----------------
namespace_argument = DeclareLaunchArgument("namespace", default_value="dyno")
vehicle_argument = DeclareLaunchArgument("vehicle", default_value="")
parameters_overrides_argument = DeclareLaunchArgument(
    "parameters_overrides",
    default_value=Path(
        get_package_share_path("dyno_simulation") /
        "config/parameters/overrides_defaults.yaml").as_posix())
log_level_argument = DeclareLaunchArgument("log_level", default_value="WARN")
robot_description_argument = DeclareLaunchArgument(
    "robot_description", default_value="olav_description")

# Nodes
# -----
# > Native simulator interface node
native_simulator_interface_node = Node(
    namespace=[
        LaunchConfiguration("namespace"), "/",
        LaunchConfiguration("vehicle")
    ],
    name="native_simulator_interface",
    package="dyno_simulation",
    executable="dyno_simulation_native_simulator_interface_node",
    arguments=["--ros-args", "--log-level",
               LaunchConfiguration("log_level")],
    parameters=[
        Path(get_package_share_path("dyno_simulation")) /
        "config/parameters/native_simulator_interface_node_defaults.yaml",
        LaunchConfiguration("parameters_overrides"),
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
        ("sensors/navigation/odometry",
         "sensors/navigation/odometry"),
        ("sensors/navigation/imu",
         "sensors/navigation/imu"),
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

# > Joystick node
joy_node = Node(
    namespace=LaunchConfiguration("namespace"),
    name="joystick_parser",
    package="joy",
    executable="joy_node",
    arguments=["--ros-args", "--log-level",
               LaunchConfiguration("log_level")],
    parameters=[
        Path(
            get_package_share_path("dyno_simulation") /
            "config/parameters/joy_node_defaults.yaml").as_posix(),
        LaunchConfiguration("parameters_overrides"),
    ],
    remappings=[("joy", "hmi/joystick/state"),
                ("joy/set_feedback", "hmi/joystick/feedback")],
    emulate_tty=True,
    output={
        "both": ["screen", "own_log"],
    },
    on_exit=Shutdown(),
)


def launch_robot_description(context: LaunchContext,
                             robot_description: LaunchConfiguration):

    # Launch descriptions
    # -------------------
    # > Robot descrption launch description
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                Path(
                    get_package_share_path(
                        context.perform_substitution(robot_description)) /
                    "launch/description.launch.py").as_posix()
            ]))
    ]


def generate_launch_description():
    """Generate the launch description."""

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        robot_description_argument,
        # > Launch configurations
        OpaqueFunction(function=launch_robot_description,
                       args=[LaunchConfiguration("robot_description")]),
        # > Nodes
        native_simulator_interface_node,
        joy_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
