import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bug2_navigation',
            namespace='tb3_0',
            executable='go_to_point',
            name='go_to_point'),

        launch_ros.actions.Node(
            package='bug2_navigation',
            namespace='tb3_0',
            executable='wall_follower',
            name='wall_follower'),

        launch_ros.actions.Node(
            package='bug2_navigation',
            namespace='tb3_0',
            executable='bug2_controller',
            name='bug2_controller'),
    ])
