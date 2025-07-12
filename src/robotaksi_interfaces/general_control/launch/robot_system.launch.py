import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Model yolu, eğer TrafficSignDetector için gerekiyorsa
    model_launcher_share_dir = get_package_share_directory('model_launcher')
    model_path = os.path.join(model_launcher_share_dir, 'models', 'model.pt') # Modelinizin yolu

    ld = LaunchDescription()

    # 1. TrafficSignDetector Node
    traffic_sign_detector_node = Node(
        package='model_launcher', # Bu düğümün olduğu paketin adı
        executable='detector_node', # Yukarıda oluşturduğunuz düğümün adı
        name='traffic_sign_detector',
        output='screen',
        parameters=[
            {'model_path': model_path},
            {'camera_topic': '/camera_rgb/image_raw'},
            {'depth_camera_topic': '/camera_depth/depth/image_raw'},
            {'display_image': True},
            {'confidence_threshold': 0.5},
            {'iou_threshold': 0.45},
            {'pedestrian_crossing_trigger_distance': 10.0},
            {'bus_stop_trigger_distance': 10.0} # Durak için daha yakın bir mesafe olabilir
        ]
    )

    # 2. RobotStateController Node
    robot_state_controller_node = Node(
        package='general_control', # Bu düğümün olduğu paketin adı
        executable='robot_state_controller', # Yukarıda oluşturduğunuz düğümün adı
        name='robot_state_controller',
        output='screen',
        parameters=[
            {'bus_stop_signal_topic': '/bus_stop_detected_signal'},
            {'stop_sign_algorithm_completion_topic': '/stop_sign_algorithm_completed'},
            {'robot_current_state_topic': '/robot_current_state'}
        ]
    )

    # 3. StopSignAlgorithm Node
    stop_sign_algorithm_node = Node(
        package='durak_controller', # Bu düğümün olduğu paketin adı
        executable='stop_sign_node', # Yukarıda oluşturduğunuz düğümün adı
        name='stop_sign_algorithm',
        output='screen',
        parameters=[
            {'robot_current_state_topic': '/robot_current_state'},
            {'stop_sign_algorithm_completion_topic': '/stop_sign_algorithm_completed'},
            {'algorithm_duration_sec': 3.0} # Örnek süre, ayarlayın
        ]
    )

    # 4. RobustLaneDetector Node
    robust_lane_detector_node = Node(
        package='lane_follower', # Bu düğümün olduğu paketin adı
        executable='lane_follower', # RobustLaneDetector için uygun bir isim
        name='lane_follower',
        output='screen',
        parameters=[
            {'roi_height_ratio': 0.6},
            {'roi_width_ratio': 0.8},
            {'min_line_angle': 20.0},
            {'lane_width_pixels': 300}
        ]
    )

    # 5. LaneControlPID Node (veya LaneControlDirect - ikisini aynı anda çalıştırmayın)
    lane_control_pid_node = Node(
        package='lane_follower', # Bu düğümün olduğu paketin adı
        executable='lane_control_pid',
        name='lane_control_pid',
        output='screen',
        parameters=[
            {'kp': 0.5},
            {'ki': 0.02},
            {'kd': 0.1},
            {'max_output': 0.5},
            {'base_speed': 0.5},
            {'min_speed': 0.2},
            {'timeout_sec': 1.0}
        ]
    )

    # 6. TwistFilterNode (Şimdilik isteğe bağlı, eğer gerçekten gerekiyorsa ve Twist mesajları kullanıyorsanız)
    # Eğer LaneControlPID ve StopSignAlgorithm doğrudan joint/velocity komutları yayınlıyorsa bu düğüme ihtiyacınız yok.
    # twist_filter_node = Node(
    #     package='your_base_control_package',
    #     executable='twist_filter_node',
    #     name='twist_filter_node',
    #     output='screen',
    #     parameters=[
    #         # ... (TwistFilterNode parametreleri)
    #     ]
    # )

    ld.add_action(traffic_sign_detector_node)
    ld.add_action(robot_state_controller_node)
    ld.add_action(stop_sign_algorithm_node)
    ld.add_action(robust_lane_detector_node)
    ld.add_action(lane_control_pid_node)
    # ld.add_action(twist_filter_node) # Eğer kullanmıyorsanız bu satırı yorumda bırakın

    return ld