

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "fleet_name",
            default_value="turtlebot",
            description='The name of the fleet.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robots",
            default_value=["tb4"],
            description="The names of robots in this fleet.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "navgraph_path",
            default_value=os.path.join(get_package_share_directory("rmf_demos_maps"), "maps/office/nav_graphs/0.yaml "),
            description="The path to the navigation graph for this fleet.",

        )
    )

    # Initialize Arguments
    fleet_name = LaunchConfiguration("fleet_name")
    robots = LaunchConfiguration("robots")
    navgraph_path = LaunchConfiguration("navgraph_path")

    adapter = Node(
        namespace="",
        package="fleet_adapter_tb",
        executable="full_control",
        name="fleet_adapter_tb",
        parameters=[
          {"fleet_name": fleet_name},
          # {"robots": robots},
          {"navgraph_path": navgraph_path}
        ],
        output="screen",
    )

    envar_console_output = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
    )

    envar_colorized_output = SetEnvironmentVariable(
        'RCUTILS_COLORIZED_OUTPUT', '1'
    )

    nodes = [
        adapter,
        envar_console_output,
        envar_colorized_output,
    ]

    return LaunchDescription(declared_arguments + nodes)


def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
