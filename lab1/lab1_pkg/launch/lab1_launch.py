from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments for parameters
    talker_v_arg = DeclareLaunchArgument(
        'v',
        default_value='1.0',
        description='Velocity parameter for the talker node'
    )
    
    talker_d_arg = DeclareLaunchArgument(
        'd',
        default_value='0.0',
        description='Steering angle parameter for the talker node'
    )

    # Define the talker node
    talker_node = Node(
        package='lab1_pkg',  
        executable='talker',  # Make sure the executable name matches the compiled node
        name='talker',
        output='screen',
        parameters=[{'v': LaunchConfiguration('v')},
                    {'d': LaunchConfiguration('d')}]
    )

    # Define the relay node
    relay_node = Node(
        package='lab1_pkg',  
        executable='relay',  # Make sure the executable name matches the compiled node
        name='relay',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the declared arguments and nodes to the launch description
    ld.add_action(talker_v_arg)
    ld.add_action(talker_d_arg)
    ld.add_action(talker_node)
    ld.add_action(relay_node)

    return ld
