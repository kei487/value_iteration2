import launch, os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition


def generate_launch_description():
    emcl2_share_dir = get_package_share_directory("emcl2")
    vi_share_dir = get_package_share_directory("value_iteration2")

    map_type = LaunchConfiguration('map_type')

    declare_map_type = DeclareLaunchArgument(
        'map_type',
        default_value='occupancy',
        description='Map type for value iteration: occupancy or cost')

    config = os.path.join(
      get_package_share_directory('value_iteration2'),
      'config',
      'params.yaml'
    )

    vi_node = Node(
            package='value_iteration2',
            namespace='value_iteration2',
            executable='vi_node',
            name='vi_node',
            parameters=[config, {'map_type': map_type}],
        )

    emcl2_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("emcl2"),
                emcl2_share_dir + "/test/nonavigation.launch.xml",
            )
        )
    )

    cost_map_yaml = os.path.join(vi_share_dir, 'maps', 'cost.yaml')
    cost_map_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["'", map_type, "' == 'cost'"])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='cost_map_server',
                parameters=[{'yaml_filename': cost_map_yaml}],
                remappings=[('map', 'cost_map_for_vi')],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_cost_map',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': ['cost_map_server']}]),
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', vi_share_dir + '/config/config.rviz'])

    return launch.LaunchDescription([
        declare_map_type,
        emcl2_launch,
        cost_map_nodes,
        vi_node,
        rviz,
    ])
