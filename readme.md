# Description
This package contains a template ros node that transforms an incoming ROS2 pointcloud2 into a PCL pointcloud2.      
PCL can then be used to process the pointcloud.    
After sucessfull processing the pointcloud, it is turned into a second ROS2 pointcloud2 and is published.       
    
# Bug report
None known

# Build the package      
Ubuntu 20.04 and ROS2 Foxy is required for this package.      


Clone this repository in your ros2 workspace (most commonly `$/home/$USER/dev_ws`)     
  
    
Install pcl_conversions to convert from ROS2 pointcloud2 to PCL pointcloud2    

```
sudo DEBIAN_FRONTEND=noninteractive apt-get install ros-foxy-pcl-conversions -y
```
   

Source ROS2 in your terminal   

```
source /opt/ros/foxy/setup.bash
```

Build the package with colcon.
```
cd /home/$USER/dev_ws && 
colcon build --packages-select pcl_example --event-handlers console_direct+ &&
cd ..
```

# Run the node 

In your ros2 sourced terminal session call:    
    
``` 
ros2 run pcl_example pcl_example_node 
``` 
    
To customize parameter call the nodes with custom arguments. E.g.:  

``` 
ros2 run pcl_example pcl_example_node --ros-args --remap topic_pointcloud_in:=bf_lidar/point_cloud_out --remap topic_pointcloud_in:=bf_lidar/point_cloud_pcl_example
```  

You can also use the launch file supplied with the package to start the nodes. 
From your ros2 development directory call:    

```  
cd /home/$USER/dev_ws/src/pcl_example &&   
ros2 launch launch/launch.py && 
cd ../../..
```  

Mind that folders that are called "launch" inside your main
work directory leads to a KeyError when executing `ros2 launch ..`.     
     
# Launchtests

Run all test defined in the package by following command 
 
```
cd /home/$USER/dev_ws && 
colcon test --packages-select pcl_example --event-handlers console_direct+ &&
cd ..
```     

Also you can manually launch the launchtests defined in this repository in the terminal     
    
```  
cd /home/$USER/dev_ws/src/pcl_example && 
launch_test test/pcl_example_launch.testing.py && 
cd ../.. /..
```  
   
# VS Code
If the "IDE" VS Code is used, build and test tasks can be called directly from the console    
Open the package in VS Code 
```
cd /home/$USER/dev_ws/src/pcl_example &&
code . &&
cd ../../.. 
```
With Shift+Control+B the package can be build.    
 
With Shift+Control+P -> Task: Run Test Task -> test  tests can be run.       

