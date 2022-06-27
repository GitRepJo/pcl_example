// Copyright (c) 2022 Jonas Mahler

// This file is part of pcl_example.

// pcl_example is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with Foobar. If not, see <https://www.gnu.org/licenses/>. 

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "pcl_example/pcl_example_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
    
Pcl_Example::Pcl_Example(const rclcpp::NodeOptions& options) : Node("pcl_example",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_pcl_example");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 10, std::bind(&Pcl_Example::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       pcl_example\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    No filter applied in this example.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str());

}

void Pcl_Example::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  unsigned int num_points = msg->width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // ROS2 Pointcloud2 to PCL Pointcloud2
  
  pcl_conversions::toPCL(*msg,*cloud);    
                       
  // Insert your pcl object here
  // -----------------------------------

  cloud_filtered = cloud;

  //------------------------------------

  // PCL message to ROS2 message 

  sensor_msgs::msg::PointCloud2 cloud_out;

  pcl_conversions::fromPCL(*cloud_filtered,cloud_out);  

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the pcl_example_out pointcloud is %i", num_points_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(cloud_out);
}
