import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

num_robots = 2
for arg in sys.argv:
    if arg.startswith("num_robots:="):
        num_robots = int(arg.split(":=")[1])

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarmosa_description').find('swarmosa_description')
    default_model_path = os.path.join(pkg_share, 'src/description/swarmosa.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    spawners = []
    ld = launch.LaunchDescription()
    ld.add_action(launch_ros.actions.SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'))
    ld.add_action(launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'))
    ld.add_action(robot_state_publisher_node)

    for robots in range(int(num_robots)):
        ld.add_action(
            launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'swarmosa' + str(robots), 
                    '-topic', 'robot_description',
                    '-robot_namespace', 'swarmosa' + str(robots),
                    '-x', '1.0',
                    '-y', str(robots)],
            output='screen'
            )
        )

    return ld