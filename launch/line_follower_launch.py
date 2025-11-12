import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('line_follower')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'e-puck_line_follower.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='e-puck',
        output='screen',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    line_follower_node = Node(
        package='line_follower',
        executable='line_follower_node',
        output='screen',
        #parameters=[os.path.join(package_dir, 'config', 'line_follower_params.yaml')]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        line_follower_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])