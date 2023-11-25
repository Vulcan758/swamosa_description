import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world = '/home/mahir/swarm_ws/src/swarmosa_description/worlds/big_house.world'
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarmosa_description').find('swarmosa_description')
    # default_model_path = os.path.join(pkg_share, 'src/description/anotha_one.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_model_path = os.path.join(pkg_share, 'urdf/swarmosa_main.urdf')
    #world = os.path.join(get_package_share_directory(
        # robot_name), 'worlds', world_file_name)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'swarmosa', '-topic', 'robot_description'],
        output='screen'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )


    return launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', world,
                 '-s', 'libgazebo_ros_factory.so'], 
                 output='screen'),
        #joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        spawn_entity,
        rviz_node
    ])