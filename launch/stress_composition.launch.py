from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  mb = LaunchConfiguration('mb')
  hz = LaunchConfiguration('hz')
  best_effort = LaunchConfiguration('best_effort')
  use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
  multi_sub = LaunchConfiguration('multi_sub')

  nodes = [
    ComposableNode(
      package='stress',
      plugin='StressPublisher',
      name='talker',
      parameters=[{'size_mb': mb},
                  {'hz': hz},
                  {'best_effort': best_effort}],
      extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]
    ),
    ComposableNode(
      package='stress',
      plugin='StressSubscriber',
      name='listener0',
      parameters=[{'best_effort': best_effort}],
      extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}]
    )
  ];

  multi_sub_nodes = nodes[:]
  for i in range(9):
    multi_sub_nodes += [ComposableNode(
      package='stress',
      plugin='StressSubscriber',
      name='listener' + str(i + 1),
      parameters=[{'best_effort': best_effort}],
      extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
    )]

  return LaunchDescription([
    DeclareLaunchArgument('mb', default_value='10'),
    DeclareLaunchArgument('hz', default_value='10'),
    DeclareLaunchArgument('best_effort', default_value='false'),
    DeclareLaunchArgument('use_intra_process_comms', default_value='true'),
    DeclareLaunchArgument('multi_sub', default_value='false'),
    ComposableNodeContainer(
      name='container',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=nodes,
      condition=UnlessCondition(multi_sub)),
    ComposableNodeContainer(
      name='container',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=multi_sub_nodes,
      condition=IfCondition(multi_sub))
  ])
