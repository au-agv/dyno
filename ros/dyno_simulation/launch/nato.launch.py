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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration


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

    # Launch descriptions
    # -------------------
    nato_group_action = GroupAction(actions=[
        PushRosNamespace(LaunchConfiguration("namespace")),
        PushRosNamespace(LaunchConfiguration("vehicle")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                Path(
                    get_package_share_path("avt_341") /
                    "launch/avt_341.launch.py").as_posix()),
            launch_arguments={
                "namespace":
                "autonomy",
                "topic/in/odometry": [
                    "/", "dyno", "/",
                    LaunchConfiguration('vehicle'), "/",
                    "sensors/navigation/odometry"
                ],
                "topic/in/steering_angle": [
                    "/", "dyno", "/",
                    LaunchConfiguration('vehicle'), "/",
                    "sensors/steering/angle"
                ],
                "topic/in/lidar": [
                    "/", "dyno", "/",
                    LaunchConfiguration('vehicle'), "/", "sensors/lidar/points"
                ],
                "topic/in/camera": [
                    "/", "dyno", "/",
                    LaunchConfiguration('vehicle'), "/",
                    "sensors/camera/mono/image/raw"
                ],
                "topic/out/drive": [
                    "/", "dyno", "/",
                    LaunchConfiguration('vehicle'), "/", "controls/drive"
                ],
            }.items())
    ], )

    return LaunchDescription([
        # > Launch arguments
        namespace_argument,
        vehicle_argument,
        log_level_argument,
        parameters_overrides_argument,
        # > Group actions
        nato_group_action
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()


if __name__ == '__main__':
    main()
