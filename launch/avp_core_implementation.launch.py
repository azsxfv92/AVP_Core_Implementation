from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    main_node = Node(
        package='avp_core_implementation',  
        executable='avp_main_node',         
        name='avp_main_node',
        output='screen',
    )

    ctrl_node = Node(
        package='avp_core_implementation',
        executable='avp_controller_node',
        name='avp_controller_node',
        output='screen'
    )

    return LaunchDescription([main_node, ctrl_node])