# An example of triggering managed node transitions from launch file

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='package_name', executable='a_managed_node', name='node1'),
        LifecycleNode(package='package_name', executable='a_managed_node', name='node2'),
        LifecycleTransition(
            lifecycle_node_names=['node1','node2'],
            transition_ids=[
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE
            ]
        )
    ])