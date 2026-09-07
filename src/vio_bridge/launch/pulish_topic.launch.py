import launch
import launch_ros

def generate_launch_description():
    action_imu_topic = launch_ros.actions.Node(
        package='vio_bridge',
        executable='vio_bridge_node',
        output='screen'
    )

    action_camera_topic = launch_ros.actions.Node(
        package='vio_bridge',
        executable='camera_node',
        output='screen'
    )

    launch_description = launch.LaunchDescription(
        [
            action_imu_topic,
            action_camera_topic
        ]
    )

    return launch_description