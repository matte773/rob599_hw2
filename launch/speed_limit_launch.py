import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='twistpublisher',
            name='twist_publisher',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='speedlimiter',
            name='speed_limiter',
            output='screen',
            parameters=[
                {'linear_max': 10.0},  # Adjust the parameter values as needed
                {'angular_max': 10.0}
            ]
        ),

        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='twistchecker',
            name='twist_checker',
            output='screen',
            parameters=[
                {'linear_max': 10.0},  # Same parameter values as the SpeedLimiter node for consistency
                {'angular_max': 10.0}
            ]
        )
    ])
