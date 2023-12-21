from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Locationの設定＆マップの読み込み
    map_name = DeclareLaunchArgument('map_name', default_value='rcj_2022_yumeko1', description='Map name to load')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='location',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_name')}]
    )

    # AMCL
    amcl_launch_file_path = 'path_to_your_amcl_launch_file'
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file_path),
        launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
    )

    # Move base
    move_base_launch_file_path = os.path.join(
        os.path.dirname(__file__), 'move_base_launch.py'  # move_base_launch.pyのパスを指定
    )
    move_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_base_launch_file_path),
        launch_arguments={'map_name': LaunchConfiguration('map_name')}.items()
    )

    # navi location server
    navi_location_server = Node(
        package='happymimi_navigation',
        executable='navi_location.py',
        name='navi_location_server',
        output='screen'
    )
    
    # navi coord server
    navi_coord_server = Node(
        package='happymimi_navigation',
        executable='navi_coord.py',
        name='navi_coord_server',
        output='screen'
    )

    return LaunchDescription([
        map_name,
        map_server,
        amcl,
        move_base,
        navi_location_server,
        navi_coord_server
    ])
