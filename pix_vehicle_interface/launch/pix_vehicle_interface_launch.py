from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pix_vehicle_interface',
            executable='pix_interface_rpt',
            name='pix_interface_report_node'
        ),
        Node(
            package='pix_vehicle_interface',
            executable='pix_interface_cmd',
            name='pix_interface_command_node'
        )
    ])
