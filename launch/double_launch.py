import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarmosa_description').find('swarmosa_description')
    default_model_path = os.path.join(pkg_share, 'src/description/swarmosa.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    # )
    spawn_entity_1 = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'swarmosa1', 
                   '-topic', 'robot_description',
                   '-robot_namespace', 'swarmosa1',
                   '-x', '1.0',
                   '-y', '1.0'],
        output='screen'
    )    
    spawn_entity_2 = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'swarmosa2', 
                   '-topic', 'robot_description',
                   '-robot_namespace', 'swarmosa2',
                   '-x', '1.0',
                   '-y', '2.0'],        
        output='screen'
    )


    return launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        # joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity_1,
        spawn_entity_2,
    ])