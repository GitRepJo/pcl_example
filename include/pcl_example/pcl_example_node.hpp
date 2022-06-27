// Copyright (c) 2022 Jonas Mahler

// This file is part of pcl_example.

// pcl_example is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

#ifndef PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_
#define PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"

/**
 * @class pcl_example::Pcl_Example
 * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
 * 
 */
class Pcl_Example : public rclcpp::Node
{
  public:
    
    /**
     * @brief A constructor for pcl_example::Pcl_Example class
     * @param options Additional options to control creation of the node.
     */
    explicit Pcl_Example(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for pcl_example::Pcl_Example class
     */
    ~Pcl_Example() {};

  protected:
    /**
     * @brief Use a no filter of pcl library
     * @param msg Pointcloud2 message receveived from the ros2 node
     * @return -
     * @details Omit pointcloud filtering in this example
     */
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string param_topic_pointcloud_out;
    
};

#endif //PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_