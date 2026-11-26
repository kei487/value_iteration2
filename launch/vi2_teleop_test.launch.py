import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vi_share = get_package_share_directory('value_iteration2')

    map_file = os.path.join(vi_share, 'maps', 'map.yaml')
    vi_params = os.path.join(vi_share, 'config', 'params.yaml')
    planner_params = os.path.join(vi_share, 'config', 'planner_params.yaml')
    rviz_config = os.path.join(vi_share, 'config', 'teleop_test.rviz')

    use_teleop = LaunchConfiguration('use_teleop')
    initial_x = LaunchConfiguration('x')
    initial_y = LaunchConfiguration('y')
    initial_yaw = LaunchConfiguration('yaw')

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch teleop_twist_keyboard in this process (needs a TTY)')
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file}],
        output='screen',
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}],
    )

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    fake_odom = Node(
        package='value_iteration2',
        executable='fake_odom.py',
        name='fake_odom',
        parameters=[{
            'x': ParameterValue(initial_x, value_type=float),
            'y': ParameterValue(initial_y, value_type=float),
            'yaw': ParameterValue(initial_yaw, value_type=float),
        }],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_teleop),
    )

    vi_node = Node(
        package='value_iteration2',
        namespace='value_iteration2',
        executable='vi_node',
        name='vi_node',
        parameters=[vi_params],
        remappings=[('/cmd_vel', '/cmd_vel_vi')],
        output='screen',
    )

    planner_node = Node(
        package='value_iteration2',
        namespace='value_iteration2',
        executable='vi_planner',
        name='vi_planner',
        parameters=[planner_params],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    usage = LogInfo(msg=[
        'vi2_teleop_test: drive with teleop (i/j/l/, keys). ',
        'Set goal in RViz (2D Goal Pose). ',
        'VI cmd_vel is remapped to /cmd_vel_vi (ignored). ',
        'If teleop has no keyboard focus, run: ',
        'ros2 run teleop_twist_keyboard teleop_twist_keyboard'
    ])

    return LaunchDescription([
        declare_use_teleop,
        declare_x,
        declare_y,
        declare_yaw,
        usage,
        map_server,
        lifecycle_manager,
        static_map_odom,
        fake_odom,
        teleop,
        vi_node,
        planner_node,
        rviz,
    ])
