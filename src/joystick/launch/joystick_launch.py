from launch import LaunchDescription
from launch_ros.actions import Node


# Simultaneously launches the `joy` and tank drive publisher nodes to receive,
# process, and send controller input to the rover.
def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        emulate_tty=True,
        output='screen'
    )

    tank_drive_publisher_node = Node(
        package='joystick',
        executable='tank_drive_publisher',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        tank_drive_publisher_node
    ])
