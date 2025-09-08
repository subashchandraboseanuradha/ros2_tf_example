from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('my_robot_bringup'),'config','joystick.yaml')
    # If running twist_mux separately, no need for mux params here

    # Print a message about the time source being used
    log_time_source = LogInfo(
        msg=["Joystick controller using use_sim_time: ", use_sim_time]
    )

    # Declare use_sim_time parameter
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true, real time if false')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            output='screen',
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')],
            output='screen',
         )

    
   

    # For Humble, we might need the twist_stamper if using stamped velocity commands
    # Uncomment if needed later
    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )


    return LaunchDescription([
        declare_use_sim_time,
        log_time_source,
        joy_node,
        teleop_node,
        # twist_stamper       
    ])