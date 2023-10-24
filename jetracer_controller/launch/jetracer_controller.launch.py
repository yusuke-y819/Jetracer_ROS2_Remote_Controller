from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    # 起動したいノードを記述
    Node(
      package='jetracer_controller',
      executable='logicool_handle'
    ),
    Node(
      package='jetracer_controller',
      executable='velocity_selector'
    )
  ])
