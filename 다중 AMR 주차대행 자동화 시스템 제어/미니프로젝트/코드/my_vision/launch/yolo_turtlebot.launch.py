from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot5',
        description='Robot name (robot0, robot5, etc.)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='best.pt',
        description='Path to YOLO model file (best.pt)'
    )
    
    use_preview_arg = DeclareLaunchArgument(
        'use_preview',
        default_value='false',
        description='Use preview image (lower resolution, faster)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='YOLO confidence threshold (0.0-1.0)'
    )

    # YOLO Node
    yolo_node = Node(
        package='my_vision',
        executable='yolo_oakd_subscriber',
        name='yolo_oakd_detector',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'model_path': LaunchConfiguration('model_path'),
            'use_preview': LaunchConfiguration('use_preview'),
            'confidence': LaunchConfiguration('confidence'),
            'topic_annotated': 'yolo/image_annotated',
            'topic_result': 'yolo/result_text'
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        model_path_arg,
        use_preview_arg,
        confidence_arg,
        yolo_node
    ])
