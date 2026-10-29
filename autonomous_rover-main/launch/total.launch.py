import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    launch_1=IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(),"/startup.launch.py"]),
        launch_arguments={
            "use_sim_time":"false"
        }.items()        
        )
    lidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'), # Check if your pkg name is exactly this
        'launch',
        'sllidar_a1_launch.py'
    )

    launch_2 = IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            "channel_type":"channel_type",
            "serial_port": "serial_port",
            "serial_baudrate": "serial_baudrate",
            "frame_id":"laser_frame",
            "inverted":"inverted",
            "angle_compensate": "angle_compensate"
        

        }.items()                                
        )    
    launch_files = [launch_1, launch_2]
    return LaunchDescription(launch_files)