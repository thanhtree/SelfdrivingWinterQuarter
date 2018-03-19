# ROS

This is the directory for the Data Collection package. The Data Collection package will publish sensor and algorithm values (i.e. from the RPLidar device and Lane Line Detection algorithm) for other system components to subscribe to.

## Install

### ROS

Our code is written for ROS Kinetic Kame on Ubuntu 16.04 LTS. To install ROS, follow the instructions at
http://wiki.ros.org/kinetic/Installation/Ubuntu.


### Data Collection Package

Assuming your catkin workspace is called ```catkin_ws```, place the ```data_collection``` and ```rplidar_ros``` folders to the ```catkin_ws/src``` directory, so that your file structure looks like this:

```
catkin_ws/
  build/
  devel/
  install/
  src/
    CMakeLists.txt
    data_collection/
    rplidar_ros/

```

## Running the Data Collection Nodes

1. In your catkin workspace top-level directory, run ```$ catkin_make```.

2. In the same workspace directory, run ```source devel/setup.bash```.

You should now be able to run ROS commands, such as ```$ rosrun data_collection lane_line_center_offset_publisher.py``` (do not run this yet). A good way to test if your commands are working is to first type ```$ rosrun data_``` and then press TAB. The command should autocomplete with ```$ rosrun data_collection```.

3. In a separate terminal, run ```$ roscore``` to set up the master node

4. In other terminals, run the following (1 command for each terminal)

```
$ rosrun data_collection lane_line_center_offset_publisher.py
```
```
$ rosrun data_collection lane_line_radius_of_curvature_publisher.py
```
```
$ rosrun data_collection subscriber_node.py
```

In summary, you should have: 
- 1 terminal running the master node 
- 1 terminal printing the lane line center offset values (lane_line_center_offset_publisher.py)
- 1 terminal printing the lane line radius of curvature values (lane_line_radius_of_curvature_publisher.py)
- 1 terminal listening and printing both values (subscriber_node.py)

## Image Transport

On Raspberry Pi, you will need to install the following packages in the ```~/catkin_ws/src``` directory. 

```$ git clone https://github.com/ros-perception/image_common```
```$ git clone https://github.com/ros-perception/vision_opencv```

Installation:  http://wiki.ros.org/image_transport/Tutorials/PublishingImages
Usage: http://wiki.ros.org/image_transport/Tutorials/ExaminingImagePublisherSubscriber

## Running ROS on multiple machines

To run ROS on multiple machines, see http://wiki.ros.org/ROS/Tutorials/MultipleMachines. Multiple machines can publish data over the network by designating one machine as the master and running ```export ROS_MASTER_URI=user@address:port``` on each machine. 

This way, each machine will know that user@address:port is the master node. Only the master machine needs to run ```$ roscore```. Otherwise, other machines can test if they are subscribed and configured properly by running ```$ rosnode list```. If the master machine has published topics but other machines cannot see them, try checking the ```ROS_MASTER_URI``` value again. 
