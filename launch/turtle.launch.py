import launch, os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    emcl2_share_dir = get_package_share_directory("emcl2")
    vi_share_dir = get_package_share_directory("value_iteration2")

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
            parameters=[config],
        )

    emcl2_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("emcl2"),
                emcl2_share_dir + "/test/nonavigation.launch.xml",
            )
        )
    )
    
    planner_node = Node(
            package='value_iteration2',
            namespace='ike_nav',
            executable='planner',
            #output='screen'
            parameters=[{
                'use_dijkstra': False,
                'publish_searched_map': True,
                'update_path_weight': 0.05,
                'smooth_path_weight': 0.8,
                'iteration_delta_threshold': 1.e-6,
            }],
            #extra_arguments=[{'use_intra_process_comms': False}],
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', vi_share_dir + '/config/config.rviz'])

    return launch.LaunchDescription([
        emcl2_launch,
        vi_node,
        planner_node,
        rviz,
    ])
