# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessStart,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import (
    ParameterFile,
)
from launch_ros.substitutions import FindPackageShare
from tr1200_modules.launch import (
    DeclareInterbotixTR1200RobotDescriptionLaunchArgument,
)


def launch_setup(context, *args, **kwargs):

    robot_description_launch_arg = LaunchConfiguration('robot_description')
    robot_name_launch_arg = LaunchConfiguration('robot_name').perform(context)

    if robot_name_launch_arg == '':
        base_topic_namespace = ''
    else:
        base_topic_namespace = f'/{robot_name_launch_arg}'

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('tr1200_control'),
            'config',
            'tr1200_controllers.yaml',
        ]),
        allow_substs=True
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_control_controllers_config_parameter_file,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/tr1200_controller/cmd_vel_unstamped', f'{base_topic_namespace}/cmd_vel'),
            ('/tr1200_controller/odom', f'{base_topic_namespace}/odom'),
        ],
        output={'both': 'screen'},
    )

    spawn_tr1200_controller_node = Node(
        name='diff_drive_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tr1200_controller',
            '--controller-manager', 'controller_manager',
        ],
        output={'both': 'screen'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 'controller_manager',
        ],
        output={'both': 'screen'},
    )

    description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tr1200_description'),
                'launch',
                'description.launch.py'
            ]),
        ),
        launch_arguments={
            'robot_description': robot_description_launch_arg,
        }.items(),
    )

    return [
        controller_manager_node,
        description_launch_include,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    spawn_joint_state_broadcaster_node,
                    spawn_tr1200_controller_node,
                ]
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description=(
                'Name of the robot. This namespaces the controller node topics like `cmd_vel` '
                'and `odom`.'
            ),
        )
    )
    declared_arguments.append(
        DeclareInterbotixTR1200RobotDescriptionLaunchArgument()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
