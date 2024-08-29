from launch import LaunchDescription
from launch_ros.actions import Node

# run this launch file with the following command:
# ros2 launch ~/ece569-fall2024/Lab1/Chatterbot/chatterbot.launch.py

def generate_launch_description():

    
    talker1 = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker1',
        remappings=[('chatter', 'chatter1')]
    )

    listener1 = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener1',
        remappings=[('chatter', 'chatter1')]
    )
    
    launch_description = LaunchDescription([
        talker1,
        listener1,
    ])

    return launch_description