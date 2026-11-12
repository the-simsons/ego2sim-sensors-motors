from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    path = Path(get_package_share_directory('ego2sim_backend'))/'launch/backend.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                            launch_arguments={'mock':'true'}.items())])
