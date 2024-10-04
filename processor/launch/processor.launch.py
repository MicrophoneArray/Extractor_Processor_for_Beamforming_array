from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # start the extractor node
        Node(
            package='extractor_node', 
            executable='extractor',  
            name='extractor_node',
            output='screen',
        ),
        # start the beamforming node
        Node(
            package='processor',  
            executable='beamforming_node', 
            name='beamforming_node',
            output='screen',
        ),
    ])
