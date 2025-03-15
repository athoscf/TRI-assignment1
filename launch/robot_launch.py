import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('firstrobot_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    # Controlador para o segundo robô (my_robot_2)
    my_robot_2_driver = WebotsController(
        robot_name='my_robot_2',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        remappings=[
            ('/cmd_vel', '/robot2/cmd_vel'),
            ('/left_sensor', '/robot2/left_sensor'),
            ('/right_sensor', '/robot2/right_sensor')
        ]
    )

    obstacle_avoider = Node(
        package='firstrobot_webots',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        my_robot_2_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])