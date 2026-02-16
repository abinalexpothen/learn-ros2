from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='learn_nodes_executor',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='learn_nodes',
                    plugin='learn_nodes_exercise::LearnNodesNode',
                    name='learn_nodes_node'
                ),
                ComposableNode(
                    package='learn_nodes',
                    plugin='learn_nodes_exercise::LearnNodesManaged1',
                    name='learn_nodes_managed_1'
                ),
                ComposableNode(
                    package='learn_nodes',
                    plugin='learn_nodes_exercise::LearnNodesManaged2',
                    name='learn_nodes_managed_2'
                ),
            ],
            output='screen'
        )
    ])