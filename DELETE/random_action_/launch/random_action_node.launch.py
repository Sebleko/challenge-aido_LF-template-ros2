from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            packate='random_action',
            executable='ran'
        )
    ])

<?xml version="1.0" encoding="utf-8"?>
<launch>
    <node name="random_action_node" pkg="random_action" type="random_action_node.py" />
</launch>
