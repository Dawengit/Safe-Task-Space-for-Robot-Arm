import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package path
    pkg_path = get_package_share_directory('apriltag_detection')
    
    return LaunchDescription([
        # AprilTag detection node
        Node(
            package='apriltag_detection',  # PLEASE Make sure it is consistent with the name in package.xml
            executable='apriltag_detection',  # Modify according to the actual executable file name
            name='apriltag_detector',
            parameters=[{
                'image_width': 1280,
                'image_height': 720,
                'tag_size': 0.035,
                'target_tag_ids': [0, 1, 2, 3]
            }]
        ),
        
        # RViz2 visualization node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'apriltag_safety.rviz')],
            output='screen'
        )
    ])


        # # safety controller node
        # Node(
        #     package='apriltag_detection',
        #     executable='arm_safety_controller',
        #     name='safety_controller',
        #     parameters=[
        #         {'max_speed': 0.5},  # max speed  
        #         {'safety_margin': 0.1}  # safety margin
        #     ]
        # ),
        