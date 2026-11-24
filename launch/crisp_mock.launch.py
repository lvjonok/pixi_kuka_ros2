from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # A file of its own, because the hardware driver is recognised by the name of its launch
    # file (pgrep crisp_hardware.launch.py).
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("pixi_kuka_ros2"), "launch", "crisp_hardware.launch.py"]
                    )
                ),
                launch_arguments={"mode": "mock"}.items(),
            )
        ]
    )
