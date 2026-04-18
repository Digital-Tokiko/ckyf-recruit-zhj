# 导入库
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


''' ComposableNode(
                    package='image_tools',
                    plugin='image_tools::ShowImage',
                    name='showimage',
                    remappings=[('/image', '/burgerimage')],
                    parameters=[{'history': 'keep_last'}],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
'''


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='homework_container',
            package='rclcpp_components',
            executable='component_container',
            output='both',
        ),
        LoadComposableNodes(
            target_container='homework_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='decider',
                    plugin='Decider',
                    name='Decider',
                    parameters=[{'serial': LaunchConfiguration('serial')}],
                    #parameters=[{'param1': 'value'}],
                ),
                ComposableNode(
                    package='img_process',
                    plugin='RawReceiver',
                    name='raw_receiver',
                    parameters=[{'R': 0.03},{'q' : 1.0},{'common_speed': 200.0}],
                    #parameters=[{'param1': 'value'}],
                ),
                ComposableNode(
                    package='predictor',
                    plugin='Predictor',
                    name='Predictor',
                    #parameters=[{'param1': 'value'}],
                ),
                ComposableNode(
                    package='drawer',
                    plugin='Drawer',
                    name='Drawer',
                    #parameters=[{'param1': 'value'}],
                ),
            ],
        )
    ])
