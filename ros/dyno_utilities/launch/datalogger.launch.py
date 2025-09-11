#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( D ) ( Y ) ( N ) ( O )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
#############################################################################
"""Launch file."""

from os import getcwd
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    # ----------------
    namespace_argument = DeclareLaunchArgument("namespace",
                                               default_value="dyno")
    parameters_overrides_argument = DeclareLaunchArgument(
        "parameters_overrides",
        default_value=Path(
            get_package_share_path("olav_launch") /
            "config/parameters/overrides_defaults.yaml").as_posix())
    log_level_argument = DeclareLaunchArgument("log_level",
                                               default_value="INFO")
    default_path_argument = DeclareLaunchArgument("default_path",
                                                  default_value=getcwd())

    # Nodes
    # -----
    # > Datalogger node
    datalogger_node = Node(
        namespace=LaunchConfiguration("namespace"),
        name="datalogger_node",
        package="dyno_utilities",
        #prefix="konsole -e gdb -ex=r --args",
        executable="dyno_utilities_datalogger_node",
        arguments=[
            "--ros-args", "--log-level",
            LaunchConfiguration("log_level")
        ],
        parameters=[
            Path(
                get_package_share_path("dyno_utilities") /
                "config/parameters/datalogger_node_defaults.yaml").as_posix(),
            LaunchConfiguration("parameters_overrides"),
            {
                "path": LaunchConfiguration("default_path"),
            },
        ],
        remappings=[
            # > Services
            ("start", "datalogger/start"),
            ("stop", "datalogger/stop"),
        ],
        emulate_tty=True,
        output={
            "both": ["screen", "own_log"],
        },
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        log_level_argument,
        parameters_overrides_argument,
        default_path_argument,
        # > Nodes
        datalogger_node,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
