import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('firstrobot_webots')
    robot1_description_path = os.path.join(package_dir, 'resource', 'robot1.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'robot2.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world2.wbt')
    )

    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot1_description_path},
        ],
        namespace='robot1',
    )

    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot2_description_path},
        ],
        namespace='robot2',
    )

    robot1_obstacle_avoider = Node(
        package='firstrobot_webots',
        executable='obstacle_avoider',
        namespace='robot1',
        name='robot1_obstacle_avoider',
    )

    robot2_robot_seeker = Node(
        package='firstrobot_webots',
        executable='robot_seeker',
        namespace='robot2',
        name='robot2_robot_seeker',
    )

    return LaunchDescription([
        webots,
        robot1_driver,
        robot2_driver,
        robot1_obstacle_avoider,
        robot2_robot_seeker,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])