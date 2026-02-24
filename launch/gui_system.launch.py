from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Firestore Bridge (Connects to Firebase)
        # Ensure your JSON credentials key is in the package 'share' folder!
        Node(
            package='ros2_gui_bridge',
            executable='firestore_bridge.py',
            name='firestore_bridge_node',
            output='screen'
        ),

        # 2. Brick Processor (Converts JSON Strings -> ROS Messages)
        Node(
            package='ros2_gui_bridge',
            executable='brick_processor2.py',
            name='brick_processor',
            output='screen'
        ),

        # 3. Assembly Plan/Allocator (Matches GUI Plan with Camera Reality)
        # NOTE: Ensure this node provides the 'get_assembly_plan' service
        # if using with the Supervisor.
        Node(
            package='ros2_gui_bridge',
            executable='assembly_plan.py',
            name='assembly_allocator',  # This name matches the node class
            output='screen'
        ),
    ])