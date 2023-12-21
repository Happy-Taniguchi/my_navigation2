import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータとマップの設定
    map_name = 'rcj_2022_yumeko1'
    use_map_topic = False
    scan_topic = 'scan'
    odom_topic = '/odom'
    laser_topic = 'scan'
    initial_pose_x = 0.0
    initial_pose_y = 0.0
    initial_pose_a = 0.0
    odom_frame_id = 'odom'
    base_frame_id = 'base_footprint'
    global_frame_id = 'map'
    happymimi_params_dir = get_package_share_directory('happymimi_params')
    happymimi_navigation_dir = get_package_share_directory('happymimi_navigation')

    # ノードの定義
    map_server_node = Node(
        package='map_server',
        executable='map_server',
        parameters=[os.path.join(happymimi_navigation_dir, 'maps', f'{map_name}.yaml')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_map_topic': use_map_topic},
            {'odom_model_type': 'diff'},
            {'gui_publish_rate': 10.0},
            {'laser_max_beams': 60},
            {'laser_max_range': 12.0},
            {'min_particles': 500},
            {'max_particles': 2000},
            {'kld_err': 0.05},
            {'kld_z': 0.99},
            {'odom_alpha1': 1.0},
            {'odom_alpha2': 0.2},
            {'odom_alpha3': 0.2},
            {'odom_alpha4': 1.0},
            {'laser_z_hit': 0.5},
            {'laser_z_short': 0.05},
            {'laser_z_max': 0.05},
            {'laser_z_rand': 0.5},
            {'laser_sigma_hit': 0.2},
            {'laser_lambda_short': 0.1},
            {'laser_model_type': 'likelihood_field'},
            {'laser_likelihood_max_dist': 2.0},
            {'update_min_d': 0.25},
            {'update_min_a': 0.2},
            {'odom_frame_id': odom_frame_id},
            {'base_frame_id': base_frame_id},
            {'global_frame_id': global_frame_id},
            {'resample_interval': 1},
            {'transform_tolerance': 0.2},
            {'recovery_alpha_slow': 0.001},
            {'recovery_alpha_fast': 0.01},
            {'initial_pose_x': initial_pose_x},
            {'initial_pose_y': initial_pose_y},
            {'initial_pose_a': initial_pose_a}
        ],
        remappings=[('scan', scan_topic)]
    )

    move_base_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='move_base',
        output='screen',
        parameters=[
            os.path.join(happymimi_params_dir, 'location', f'{map_name}.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'costmap_common_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'local_costmap_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'global_costmap_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'dwa_local_planner_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'move_base_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'global_planner_params.yaml'),
            os.path.join(happymimi_navigation_dir, 'param', 'navfn_global_planner_params.yaml'),
            {'global_costmap/global_frame': global_frame_id},
            {'global_costmap/robot_base_frame': base_frame_id},
            {'local_costmap/global_frame': odom_frame_id},
            {'local_costmap/robot_base_frame': base_frame_id},
            {'DWAPlannerROS/global_frame_id': odom_frame_id}
        ],
        remappings=[('odom', odom_topic), ('scan', laser_topic)]
    )

    navi_location_server_node = Node(
        package='happymimi_navigation',
        executable='navi_location.py',
        name='navi_location_server',
        output='screen'
    )

    navi_coord_server_node = Node(
        package='happymimi_navigation',
        executable='navi_coord.py',
        name='navi_coord_server',
        output='screen'
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        move_base_node,
        navi_location_server_node,
        navi_coord_server_node
    ])