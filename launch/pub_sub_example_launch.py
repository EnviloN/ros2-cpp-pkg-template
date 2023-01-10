from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publisher_node = Node(
            package='cpp_pkg_template',
            executable='dummy_publisher_cpp_node',
            name='dummy_publisher_cpp'
        )

    subscriber_node = Node(
            package='cpp_pkg_template',
            executable='dummy_subscriber_cpp_node',
            name='dummy_subscriber_cpp',
            output='screen'
        )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])
