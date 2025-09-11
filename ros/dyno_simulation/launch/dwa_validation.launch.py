#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
"""Launch file."""

from os import getcwd
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # Launch arguments
    # ----------------
    namespace_argument = DeclareLaunchArgument("namespace",
                                               default_value="dyno")
    vehicle_argument = DeclareLaunchArgument("vehicle", default_value="")
    parameters_overrides_argument = DeclareLaunchArgument(
        "parameters_overrides",
        default_value=Path(
            get_package_share_path("olav_launch") /
            "config/parameters/overrides_defaults.yaml").as_posix())
    log_level_argument = DeclareLaunchArgument("log_level",
                                               default_value="WARN")
    default_path_argument = DeclareLaunchArgument("default_path",
                                                  default_value=getcwd())

    # Launch descriptions
    # -------------------
    dyno_group_action = GroupAction(actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            Path(
                get_package_share_path("dyno_simulation") /
                "launch/native.launch.py").as_posix()),
                                 launch_arguments={
                                     "namespace": LaunchConfiguration(
                                         "namespace"),
                                     "vehicle": LaunchConfiguration("vehicle"),
                                 }.items()),
    ])

    navi_group_action = GroupAction(actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            Path(
                get_package_share_path("dyno_simulation") /
                "launch/navi.launch.py").as_posix()),
                                 launch_arguments={
                                     "namespace": LaunchConfiguration(
                                         "namespace"),
                                     "vehicle": LaunchConfiguration("vehicle"),
                                 }.items()),
    ])

    datalogger_group_action = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(
                    get_package_share_path("dyno_utilities") /
                    "launch/datalogger.launch.py").as_posix()),
            launch_arguments={
                "namespace": LaunchConfiguration("namespace"),
                "default_path": LaunchConfiguration("default_path")
            }.items()),
    ])

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        vehicle_argument,
        log_level_argument,
        parameters_overrides_argument,
        default_path_argument,
        # > Group actions
        dyno_group_action,
        navi_group_action,
        datalogger_group_action,
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
