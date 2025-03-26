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

        # Node(
        #     package='apriltag_detection',
        #     executable='UR5_safety_simulator',
        #     name='safety_simulator',
        #     parameters=[{
        #         'simulation_rate': 30.0,
        #         'safety_check_rate': 10.0,
        #         'initial_position': [0.5, 0.0, 0.7],
        #         'movement_range': 0.5,
        #         'safety_margin': 0.1
        #     }],
        #     output='screen'
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.0', '0.0', '0.0',  # X/Y/Z translation
                '0', '0', '0',        # roll/pitch/yaw
                'panda_link0',      # father frame
                'camera_frame'        # son frame
            ],
            name='camera_tf_publisher'
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


       
        