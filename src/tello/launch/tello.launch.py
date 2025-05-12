from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello',              # ← your package
            executable='tello',           # ← entry‑point name
            name='tello',                 # node name
            parameters=[{
                'tello_ip': '192.168.10.1'
            }],
            remappings=[
                ('/image_raw', 'camera')  # publish as /camera
            ],
            respawn=True,
            output='screen'
        )
    ])
