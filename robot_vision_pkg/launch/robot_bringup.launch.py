from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='best.onnx',
                              description='Path to the ONNX detection model'),
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='OpenCV camera index'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Arduino serial port'),

        Node(
            package='robot_vision_pkg',
            executable='vision_node',
            name='yolo_vision_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'camera_index': LaunchConfiguration('camera_index'),
            }],
        ),
        Node(
            package='robot_vision_pkg',
            executable='arm_action_server',
            name='arm_action_server',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
            }],
        ),
    ])
