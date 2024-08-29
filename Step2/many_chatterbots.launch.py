from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# run this launch file with the following command:
# ros2 launch ~/ece569-fall2024/Lab1/Chatterbot/many_chatterbots.launch.py

def generate_launch_description():

    # Function to generate the talker and listener nodes based on the number specified
    def generate_nodes(context):
        # Get the value of 'num_pairs' argument
        # The try block is used in case the user enters a non-integer value
        try:
            num_pairs = int(LaunchConfiguration('num_pairs').perform(context))
        except:
            # log a message if the argument couldn't be parsed
            print("[WARNING]: num_pairs argument couldn't be parsed, defaulting to 1")
            num_pairs = 1
        
        nodes = []
        MAX_PAIRS = 9

        for i in range(min(num_pairs, MAX_PAIRS)):
            talker_node = Node(
                package='demo_nodes_cpp',
                executable='talker',
                name=f'talker{i+1}',
                remappings=[('chatter', f'chatter{i+1}')]
            )
            listener_node = Node(
                package='demo_nodes_cpp',
                executable='listener',
                name=f'listener{i+1}',
                remappings=[('chatter', f'chatter{i+1}')]
            )
            nodes.append(talker_node)
            nodes.append(listener_node)
        
        return nodes
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_pairs',
            default_value='1',
            description='Number of talker and listener nodes to launch'
        ),
        OpaqueFunction(function=generate_nodes),
    ])
