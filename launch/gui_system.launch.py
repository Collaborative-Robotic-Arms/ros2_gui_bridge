from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Declare Launch Arguments ---
    # Default is "sim" mode. Can be set to "real".
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='System mode: "sim" (default) or "real".'
    )
    
    mode = LaunchConfiguration('mode')

    # Boolean expressions to evaluate the current mode
    is_real = PythonExpression(["'", mode, "' == 'real'"])
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])

    # --- 2. Firestore Bridge ---
    # Runs the same way in both modes
    firestore_node = Node(
        package='ros2_gui_bridge',
        executable='firestore_bridge.py',
        name='firestore_bridge_node',
        output='screen'
    )

    # --- 3. Brick Processor Nodes ---
    # Version A: SIM MODE
    brick_processor_sim = Node(
        package='ros2_gui_bridge',
        executable='brick_processor2.py',
        name='brick_processor',
        output='screen',
        parameters=[{'use_sim': True}],
        condition=IfCondition(is_sim)
    )

    # Version B: REAL MODE
    brick_processor_real = Node(
        package='ros2_gui_bridge',
        executable='brick_processor2.py',
        name='brick_processor',
        output='screen',
        parameters=[{'use_sim': False}],
        condition=IfCondition(is_real)
    )

    # --- 4. Assembly Plan / Allocator Nodes ---
    # Version A: SIM MODE
    assembly_allocator_sim = Node(
        package='ros2_gui_bridge',
        executable='assembly_plan.py',
        name='assembly_allocator',
        output='screen',
        parameters=[{'use_sim': True}],
        condition=IfCondition(is_sim)
    )

    # Version B: REAL MODE
    assembly_allocator_real = Node(
        package='ros2_gui_bridge',
        executable='assembly_plan.py',
        name='assembly_allocator',
        output='screen',
        parameters=[{'use_sim': False}],
        condition=IfCondition(is_real)
    )

    # --- 5. Return the LaunchDescription ---
    return LaunchDescription([
        mode_arg,
        firestore_node,
        brick_processor_sim,
        brick_processor_real,
        assembly_allocator_sim,
        assembly_allocator_real
    ])