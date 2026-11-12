from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = str(Path(get_package_share_directory('ego2sim_backend'))/'config/backend.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=config),
        DeclareLaunchArgument('mock',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('port',default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('host_processing',default_value='false',choices=['true','false']),
        Node(package='ego2sim_backend',executable='serial_bridge',namespace='ego2sim',
             name='serial_bridge',output='screen',parameters=[LaunchConfiguration('config'),
                 {'mock':ParameterValue(LaunchConfiguration('mock'),value_type=bool),
                  'port':LaunchConfiguration('port'),
                  'publish_firmware_processed':ParameterValue(PythonExpression([
                      "'",LaunchConfiguration('host_processing'),"' == 'false'"]),value_type=bool)}]),
        Node(package='ego2sim_backend',executable='sensor_processing',namespace='ego2sim',
             name='sensor_processing',output='screen',condition=IfCondition(LaunchConfiguration('host_processing')),
             parameters=[LaunchConfiguration('config')])])
