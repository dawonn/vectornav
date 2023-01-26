
#
# launch file for composable node container
#
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    con = ComposableNodeContainer(
        name='vectornav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vectornav',
                plugin='vectornav::Vectornav',
                name='vectornav',
                parameters=[PathJoinSubstitution(
                    [FindPackageShare('vectornav'),
                     'config', 'vectornav_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='vectornav',
                plugin='vectornav::VnSensorMsgs',
                name='vn_sensor_msgs',
                parameters=[PathJoinSubstitution(
                    [FindPackageShare('vectornav'),
                     'config', 'vn_sensor_msgs_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}])
            ],
        output='screen')
    
    return LaunchDescription([con])

