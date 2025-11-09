import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('line_follower')
    robot_description = pathlib.Path(os.path.join(package_dir, 'urdf', 'my_robot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'e-puck_line_follower.wbt')
    )

    my_robot_driver = Node(
        package='line_follower',
        executable='line_follower_node',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'e-puck'},
        parameters=[
            os.path.join(package_dir, 'config', 'line_follower_params.yaml')
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        #ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])