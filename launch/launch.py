#  Copyright (c) 2022 Jonas Mahler

#  This file is part of pcl_example.

#  pcl_example is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_example',
            executable='pcl_example_node',
            name='pcl_example_node',
            parameters= [
            {'frame_id': 'lidar'},
            {'topic_pointcloud_in': 'bf_lidar/point_cloud_out'},
            {'topic_pointcloud_out': 'bf_lidar/point_cloud_pcl_example'}],
        )
    ])