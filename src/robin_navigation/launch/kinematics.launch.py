import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Pakai waktu simulasi Gazebo'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Radius roda (m)'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.45',
        description='Jarak antar roda (m)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom_custom',
        description='Topik odometri yang dipakai Go-to-Point'
    )

    ik_input_topic_arg = DeclareLaunchArgument(
        'ik_input_topic',
        default_value='/input_ik',
        description='Topik input untuk node IK'
    )

    inverse_kinematic_node = Node(
        package='simple_mover',
        executable='inverse_kinematic_node',
        name='inverse_kinematic_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }],
    )

    forward_kinematic_node = Node(
        package='simple_mover',
        executable='forward_kinematic_node',
        name='forward_kinematic_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'left_joint_name': 'base_left_wheel_joint',
            'right_joint_name': 'base_right_wheel_joint',
            'odom_frame': 'odom', 
            'base_frame': 'base_footprint',  
        }],
    )

    global_coordinate_node = Node(
        package='robot_global_kinematics',
        executable='global_coordinate_node',
        name='global_coordinate_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'frame_id': 'odom',
            'child_frame_id': 'base_footprint',
            'left_joint_name': 'base_left_wheel_joint',
            'right_joint_name': 'base_right_wheel_joint',
            'publish_tf': True,
        }],
    )

    go_to_point_node = Node(
        package='robin_navigation',
        executable='go2point_node',
        name='go2point_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'K_linear': 1.0,
            'K_angular': 2.0,
            'V_MAX': 0.5,
            'W_MAX': 1.5,
            'DIST_THRESHOLD': 0.05,
            'ANGLE_THRESHOLD': 0.1,
            'ROTATE_FIRST_ANGLE': 0.5,
        }],
        remappings=[
            ('/odom_custom', LaunchConfiguration('odom_topic')),
        ],
    )

    info = LogInfo(msg=[
        '\n',
        '════════════════════════════════════════════\n',
        '  KINEMATICS STACK — Robin\n',
        '════════════════════════════════════════════\n',
        '  Node yang dijalankan:\n',
        '    - inverse_kinematic_node  (simple_mover)\n',
        '    - forward_kinematic_node  (simple_mover)\n',
        '    - global_coordinate_node  (robot_global_kinematics)\n',
        '    - go2point_node           (robin_navigation)\n',
        '\n',
        '  Cara kirim target:\n',
        '    ros2 topic pub /goal_point geometry_msgs/msg/Point "{x: 1.0, y: 0.5}" --once\n',
        '════════════════════════════════════════════\n',
    ])

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        odom_topic_arg,
        ik_input_topic_arg,

        info,

        inverse_kinematic_node,
        forward_kinematic_node,
        global_coordinate_node,
        go_to_point_node,
    ])