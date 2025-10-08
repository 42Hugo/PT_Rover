import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0','frame_id': 'odom','angle_compensate': True,'scan_mode': 'Standard'}])
  ])
