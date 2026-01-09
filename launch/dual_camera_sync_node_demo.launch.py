# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)

from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)

multi_zed_xacro_path = os.path.join(
    get_package_share_directory('ros2_replay_data'),
    'urdf',
    'zed_multi.urdf.xacro')

default_svo1_path = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'examples/dual_camera_demo',
        'front_camera.svo2'
)

default_svo2_path = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'examples/dual_camera_demo',
        'back_camera.svo2'
)

default_bag_path = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'examples/dual_camera_demo/person_walking'
)

default_yaml_config_path = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'examples/dual_camera_demo',
        'dual_camera_synchronized_topics.yaml'
)

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name_1 = LaunchConfiguration('camera_name_1')
    camera_name_2 = LaunchConfiguration('camera_name_2')
    namespace = LaunchConfiguration('namespace')
    camera_model = LaunchConfiguration('camera_model')
    svo1_path = LaunchConfiguration('svo1_path')
    svo2_path = LaunchConfiguration('svo2_path')
    bag_path = LaunchConfiguration('bag_path')
    yaml_path = LaunchConfiguration('yaml_config_path')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    sync_slop = LaunchConfiguration('sync_slop')
    seek_time_step = LaunchConfiguration('seek_time_step')
    prefix = LaunchConfiguration('prefix')

    namespace_val = namespace.perform(context)
    camera1_name_val = camera_name_1.perform(context)
    camera2_name_val = camera_name_2.perform(context)
    camera_model_val = camera_model.perform(context)
    svo1_path_val = svo1_path.perform(context)
    svo2_path_val = svo2_path.perform(context)
    bag_path_val = bag_path.perform(context)
    yaml_config_path_val = yaml_path.perform(context)
    prefix_val = prefix.perform(context)
    

    if (camera1_name_val == ''):
        camera1_name_val = 'zed1'
    
    if (camera2_name_val == ''):
        camera2_name_val = 'zed2'

    camera_type = ''
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val=='virtual'):
        camera_type = 'stereo'
    else: # 'zedxonegs' or 'zedxone4k')
        camera_type = 'mono'

    # Create the Xacro command with correct camera names
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(multi_zed_xacro_path)
    xacro_command.append(' ')
    xacro_command.append('camera_name_front:=')
    xacro_command.append(camera1_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_name_back:=')
    xacro_command.append(camera2_name_val)
    xacro_command.append(' ')
   

    # Robot State Publisher node
    # this will publish the static reference link for a multi-camera configuration
    # and all the joints. See 'urdf/zed_dual.urdf.xacro' as an example    
    rsp_name = 'state_publisher'
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )

    # RVIZ2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'rviz',
        'replay_dual_camera_svo_sync.rviz'
    )

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera1_name_val,
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # ROS 2 Component Container
    container_name = 'zed_multi_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        # Foxy does not support the isolated mode
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    info = '* Starting Composable node container: /' + namespace_val + '/' + container_name

    zed_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    # ZED Wrapper launch file - CAMERA 1 
    zed_wrapper_launch_first = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'container_name': container_name,
            'camera_name': camera1_name_val,
            'camera_model': camera_model_val,
            'svo_path': svo1_path_val,
            'namespace': namespace_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )
    
    # ZED Wrapper launch file - CAMERA 2
    zed_wrapper_launch_second = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'container_name': container_name,
            'camera_name': camera2_name_val,
            'camera_model': camera_model_val,
            'svo_path': svo2_path_val,
            'namespace': namespace_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )
    
    sync_data_replay_node = Node(
            package='ros2_replay_data',
            executable='dual_camera_sync_node',
            name='sync_data_replay_node',
            output='screen',
            parameters=[
                {'bag_path': bag_path_val},
                {'synchronized_topic_config_file': yaml_config_path_val},
                {'namespace': namespace_val},
                {'camera_name_1': camera1_name_val},
                {'camera_name_2': camera2_name_val},
                {'sync_queue_size': sync_queue_size},
                {'sync_slop': sync_slop},
                {'seek_time_step': seek_time_step},
                {'prefix': prefix_val}
            ],
        )

    return [
        rviz2_node,
        zed_container,
        multi_rsp_node,
        zed_wrapper_launch_first,
        zed_wrapper_launch_second,
        sync_data_replay_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description='Set to `False` to start only RVIZ2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'camera_name_1',
                default_value=TextSubstitution(text='zed_front'),
                description='The name of the first camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_name_2',
                default_value=TextSubstitution(text='zed_back'),
                description='The name of the first camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'namespace',
                default_value=TextSubstitution(text='zed_multi'),
                description='Current namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                default_value=TextSubstitution(text='zedx'),
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'svo1_path',
                default_value=TextSubstitution(text=default_svo1_path),
                description='The svo file path to be used to replay the svo data for the first camera'),
            DeclareLaunchArgument(
                'svo2_path',
                default_value=TextSubstitution(text=default_svo2_path),
                description='The svo file path to be used to replay the svo data for the second camera'),
            DeclareLaunchArgument(
                'bag_path',
                default_value=TextSubstitution(text=default_bag_path),
                description='The bag file path to be used to replay the bag data'),
            DeclareLaunchArgument(
                'yaml_config_path',
                default_value=TextSubstitution(text=default_yaml_config_path),
                description='The yaml file that contains topics to synchronize together'),
            DeclareLaunchArgument(
                'prefix',
                default_value=TextSubstitution(text=''),
                description='String prefix to remove to wrapper topics'),
            DeclareLaunchArgument(
                'sync_queue_size',
                default_value='500',
                description='Sync message filter queue size'),
            DeclareLaunchArgument(
                'sync_slop',
                default_value='0.05',
                description='Sync message filter slop'),
            DeclareLaunchArgument(
                'seek_time_step',
                default_value='0.2',
                description='When the system is paused, users have the possibility to explore the data by seeking/jumping "seek_time_step" increment of time (in seconds)'),
            OpaqueFunction(function=launch_setup)
        ]
    )
