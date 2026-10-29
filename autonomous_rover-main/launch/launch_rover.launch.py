import os
#imported from Articulated Robotics
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import TimerAction

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='autonomous_rover' #changed to our src name

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','startup.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    gazebo_params_file = os.path.join(
            get_package_share_directory(package_name),'config','gazebo_params.yaml'
         )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'ariees'],
    #                     output='screen')

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    diff_control_file = os.path.join(
            get_package_share_directory(package_name),'config','my_controllers.yaml'
         )



    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description' : robot_description}, diff_control_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions={controller_manager})
    

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )


    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_exit=[diff_drive_spawner]
        )
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    delay_joint_board_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_exit=[joint_broad_spawner]
        )
    )
    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner
    ])
