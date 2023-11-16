import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='go_to_point',
            namespace='tb3_1',
            name='go_to_point'),

        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='wall_follower',
            namespace='tb3_1',
            name='wall_follower'),
            
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            namespace='tb3_1',
            name='bug2_controller'),

    ])
