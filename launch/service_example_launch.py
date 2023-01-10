from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    client_node = Node(
            package='cpp_pkg_template',
            executable='dummy_client_cpp_node',
            name='dummy_client_cpp'
        )

    server_node = Node(
            package='cpp_pkg_template',
            executable='dummy_server_cpp_node',
            name='dummy_server_cpp'
        )

    return LaunchDescription([
        client_node,
        server_node
    ])
