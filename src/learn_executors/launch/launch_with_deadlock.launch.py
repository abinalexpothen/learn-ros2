from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    srv_node = Node(
        package='learn_executors',
        name='deadlock_demo_server',
        executable='deadlock_server_node',
        output='screen'
    )

    client_node = Node(
        package='learn_executors',
        name='deadlock_demo_client',
        executable='deadlock_client_node',
        output='screen'
    )

    ld.add_action(srv_node)
    ld.add_action(client_node)
    return ld