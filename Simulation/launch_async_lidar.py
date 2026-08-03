from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # TF frame so RViz2 works
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_broadcaster',
             arguments=['--frame-id', 'world', '--child-frame-id', 'map']),

        # Transmitter drone (RED) at origin, 100m altitude
        Node(package='async_lidar', executable='transmitter_node',
             name='transmitter_node', output='screen',
             parameters=[{'flight_altitude': 100.0,
                          'flight_speed': 0.0,
                          'publish_rate_hz': 10.0}]),

        # Receiver 0 (GREEN)  — East
        Node(package='async_lidar', executable='receiver_node',
             name='receiver_node_0', output='screen',
             parameters=[{'receiver_id': 0, 'start_x': 80.0,
                          'start_y': 0.0, 'flight_altitude': 50.0}]),

        # Receiver 1 (BLUE)   — North
        Node(package='async_lidar', executable='receiver_node',
             name='receiver_node_1', output='screen',
             parameters=[{'receiver_id': 1, 'start_x': 0.0,
                          'start_y': 80.0, 'flight_altitude': 50.0}]),

        # Receiver 2 (MAGENTA) — West
        Node(package='async_lidar', executable='receiver_node',
             name='receiver_node_2', output='screen',
             parameters=[{'receiver_id': 2, 'start_x': -80.0,
                          'start_y': 0.0, 'flight_altitude': 50.0}]),

        # Fusion node
        Node(package='async_lidar', executable='fusion_node',
             name='fusion_node', output='screen',
             parameters=[{'n_receivers': 3,
                          'ground_z_approx': 0.0,
                          'max_pdop_threshold': 5.0,
                          'publish_rate_hz': 5.0}]),
    ])
