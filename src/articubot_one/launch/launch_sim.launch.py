import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'walls.world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    robot_mov_node = Node(
        package='articubot_one',       # Replace with your package name if different
        executable='robot_mov.py',        # Name of your executable
        output='screen'
    )

    percept_node = Node(
        package='articubot_one',       # Replace with your package name if different
        executable='percept.py',        # Name of your executable
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        robot_mov_node,
        percept_node
    ])
