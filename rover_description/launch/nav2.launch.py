import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def deep_merge(base, override):
    """Merge override into base. Dicts merge recursively; everything else is replaced."""
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def build_params_file():
    """Stock Nav2 params + our overrides -> one file in /tmp."""
    nav2_dir = get_package_share_directory('nav2_bringup')
    rover_dir = get_package_share_directory('rover_description')

    with open(os.path.join(nav2_dir, 'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(rover_dir, 'config', 'nav2_params.yaml')) as f:
        overrides = yaml.safe_load(f)

    merged = deep_merge(params, overrides)
    out = os.path.join(tempfile.gettempdir(), 'rover_nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(merged, f)
    return out


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    params_file = build_params_file()

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/site_map.yaml'),
        description='Full path to the map yaml file',
    )

    # Nav2 nodes listen on /odom; your controller publishes it under its own name.
    # Not composed, so the remap applies to every Nav2 node.
    nav2 = GroupAction([
        SetRemap(src='odom', dst='/diff_drive_base_controller/odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'slam': 'False',
            }.items(),
        ),
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([map_arg, nav2, rviz])
