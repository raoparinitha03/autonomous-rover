import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_vision_pkg')
    default_urdf = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')

    urdf_path = LaunchConfiguration('urdf_path')

    # Command(...) returns a Substitution whose result launch would
    # otherwise try to parse as YAML -- URDF is full of ':' and '<>' that
    # breaks that parse. ParameterValue(..., value_type=str) forces it to
    # be treated as a plain string instead.
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=default_urdf,
                              description='Path to the robot URDF file'),

        # Publishes /robot_description and TF for each link based on
        # whatever joint angles joint_state_publisher_gui is currently set to.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Gives you a slider for every joint (waist, shoulder, elbow, wrist
        # roll/pitch, both fingers) so you can pose the arm by hand.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])