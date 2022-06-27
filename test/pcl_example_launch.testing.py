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

import time
import unittest
import yaml

import os
import ament_index_python

import rclpy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from launch_ros.actions import Node
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util

import pytest


@pytest.mark.launch_test
def generate_test_description():
    """
    Specify nodes or processes to launch for test.
    :param -
    :return dut [ros2 node] node to be tested (device under test)
    :return ...,... specifications for launch_testing
    Multiple nodes that are to be tested can be launched
    """
    
    # Set rviz2 configuration file
    rviz2_config =  os.path.join(ament_index_python.get_package_prefix('pcl_example'),
            'lib/pcl_example/',
            'pcl_example.rviz')
    #rviz2_config = os.path.join(ament_index_python.get_package_share_directory('pcl_example'),'pcl_example', 'pcl_example_rviz2.rviz')
    
    # dut -> device under test is the node to be tested in this example
    dut = Node(
        package='pcl_example',
        executable='pcl_example_node',
        name='pcl_example_node',
        parameters= [
            {'frame_id': 'lidar'},
            {'topic_pointcloud_in': 'bf_lidar/point_cloud_out'},
            {'topic_pointcloud_out': 'bf_lidar/point_cloud_pcl_example'},
            {'leaf_x': 0.2},
            {'leaf_y': 0.2},
            {'leaf_z': 0.2}],
    )
    # Visualize results of the text in rviz2 
    rviz2 = Node(
        package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',  rviz2_config  ]
    )

    # Publish a transfrom from base map to lidar pointcloud
    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = "0 0 0 0 0 0 map lidar".split(' ')
    )

    context = {'dut': dut,
               'rviz2': rviz2,
               'tf2' : tf2
            }
    
    return (launch.LaunchDescription([
        dut,
        rviz2,
        tf2,
        launch_testing.actions.ReadyToTest()]
        ), context
    )


class TestProcessOutput(unittest.TestCase):
    """
    Details to use this class in the context of launch_testing.
    nodes: https://github.com/ros2/launch_ros
    process: https://github.com/ros2/launch/tree/master/launch_testing
    """
    message_counter = 0
    recv_msg = []
    
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('input_output_node')

    def tearDown(self):
        self.node.destroy_node()

    def timer_callback(self):
        """
        Read multiple local pointcloud2-yaml files and publish the data from these files to ros2.
        :param -
        :return -
        Details: Place the pointcloud2-yaml files in the folder specified in input_data_path.
            Add the files to input data path_name list in the code below. Messages are then send iteratively in a loop to ros2.
            This test may take some time due to a long readin time of large pointcloud yaml files.
        """
        
        input_data = []
        
        # Specify name of file to read
        input_data_name = ["cube1_record_1.yaml","cube1_record_2.yaml"]
       
        # Make path to data
        input_data_path = os.path.join(
            ament_index_python.get_package_prefix('pcl_example'),
            'lib/pcl_example/',
            input_data_name[self.message_counter])
        
        print("read data")
        # Read input data that is send to dut
        with open(input_data_path) as f:
            input_data = yaml.safe_load(f)
        print("data ready")
        
        # Set up the Pointcloud2 message
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = Time()
        msg.header.stamp.sec = input_data['header']['stamp']['sec']
        msg.header.stamp.nanosec = input_data['header']['stamp']['nanosec']
        msg.header.frame_id = input_data['header']['frame_id']
        
        msg.height = input_data['height']        
        msg.width = input_data['width']
        
        field_x= PointField()
        field_y= PointField()
        field_z= PointField()
            
        field_x.name = str(input_data['fields'][0])
        field_x.offset = input_data['fields'][1]
        field_x.datatype = input_data['fields'][2]
        field_x.count = input_data['fields'][3]

        field_y.name = str(input_data['fields'][4])
        field_y.offset = input_data['fields'][5]
        field_y.datatype = input_data['fields'][6]
        field_y.count = input_data['fields'][7]

        field_z.name = str(input_data['fields'][8])
        field_z.offset = input_data['fields'][9]
        field_z.datatype = input_data['fields'][10]
        field_z.count = input_data['fields'][11]
        
        
        msg.fields = [field_x,field_y,field_z]
        msg.is_bigendian = input_data['is_bigendian']
        msg.point_step = input_data['point_step']
        msg.row_step = input_data['row_step']
        msg.is_dense = input_data['is_dense']

        msg.data = input_data['data']
        msg.is_dense = input_data['is_dense']
        
        # Publish message
        
        self.publisher_.publish(msg)

        # message counter must not exceed the length of input_data_name
        self.message_counter = self.message_counter +1
        if self.message_counter >= len(input_data_name):
            self.message_counter = 0
        
        # Wait till published messages can be received
        time.sleep(0.1)
    
    def listener_callback(self,msg):
        """
        Listen for a message published by dut and save to global space.
        :param msg [pointcloud2] received pointcloud from dut
        :return -
        """
        print("got into listener callback")
        #self.recv_msg.append(msg) 
        
    def test_dut_output(self, dut, proc_output):
        """
        Listen for a message published by dut and compare message to expected value.
        :param -
        :return dut [ros2 node] node to be tested (device under test)
        :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
        """
        msgs_rx = []
        # Setup for listening to dut messages       
        sub = self.node.create_subscription(
            PointCloud2,
            '/bf_lidar/point_cloud_pcl_example',
            lambda msg: msgs_rx.append(msg),
            10
        )

        # Publish data to dut
        self.publisher_ = self.node.create_publisher(PointCloud2, 'bf_lidar/point_cloud_out', 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

        try:
            # Wait until the dut transmits a message over the ROS topic           
            end_time = time.time() + 40
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                print("Messages received: "+str(len(msgs_rx)))
                
                if len(msgs_rx) >= 2:
                    break
            
            # test actual output for expected output
                           
            self.assertEqual(msgs_rx[0].width, 26803)
            self.assertEqual(msgs_rx[1].width, 26884)

        finally:
            self.node.destroy_subscription(sub)