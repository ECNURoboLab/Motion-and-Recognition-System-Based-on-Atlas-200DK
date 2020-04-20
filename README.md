# Motion and Recognition System Based on Atlas 200DK

## Overview
This repository is a ROS package developed for mobile robot equipped with Huawei Atlas 200DK. The software is designed for the desktop navigation tasks with robots which only rely on raspberry pi camera. Besides that, the software also provides the gampping and navigation function with cheap distance sensor(e.g. LiDAR).

## Dependencies
This software is build on the Robotic Operating System([ROS](https://www.ros.org/)) and [Huawei Mind Studio](https://www.huaweicloud.com/ascend/resources/Tools/0). Please check it installed first.

## Deployment

### Arduino Case

Use Arduino IDE to upload the code to Arduino DUE board. The main file is `Arduino/mobileBase_IIC/mobileBase_IIC.ino`, and choose Arduino Due(Programming Port) to write.

### Detection and Recognition Application Case

This application uses Mind Studio and DDK tools to build. Run `./app/deploy.sh $remote_host_ip $download_mode` to prepare the application environment, including compiling and deploying the ascenddk public library, and configuring EZDVPP library.

- `$remote_host_ip` is the IP address of Atlas 200DK.
- `$download_mode`(internet/local) is the method to download ezdvpp library. Please choose internet for the first time.

The offline models need to be uploaded to Atlas 200DK manually. Use scp tool to convert the model file from ubuntu server `MyModel/OfflineModel/*` to Atlas200DK `~/HIAI_DATANDMODELSET/ascend_workspace`.

### ROS Package Case 
In order to deploy the system to Atlas 200DK, clone the latest version from this repository into your catkin workspace of Atlas 200DK and compile the package using ROS.
Please use ssh connect to Atlas 200DK and input the following:

```bash
mkdir ~/catkin_atlas_ws && cd ~/catkin_atlas_ws
git clone https://github.com/
catkin_make
```

## Usage

### Desktop Navigation
The desktop navigation application consist of two parts. In order to start navigation, detection and recognition application needs to start first.  
1. Connect to Atlas 200DK with ssh and input `~/HIAI_PROJECTS/ascend_workspace/collision/out/collision` to start the application.
2. Start a new ssh connection to Atlas 200DK. Now the detection and recognition neural network is running in the background. Run the ROS package to make the robot navigating on the desktop.  

```bash
source ~/catkin_atlas_ws/devel/setup.bash
# Bring up the robot
roslaunch robot_bringup robot_bringup.launch 
# Startup the Application
rosrun robot_neural_netowork neural_network.py
```

### Gmapping and Navigation
For the gmapping application, the map should be established first with the ROS package gmapping. Start a ssh connection to Atlas 200DK and input the following:
```bash
# Choose the corresponding lidar
roslaunch robot_navigation gmapping_ydlidar_x2l.launch 
```
And then start a new ssh connection to Atlas 200DK and input `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot to create a map of given regions. The map will be saved at `~/catkin_atlas_ws/src/robot_navigation/maps`.  
After establishing the map, run the amcl to make the robot navigating in the given map by entering`roslaunch robot_amcl robot_amcl.launch`.

## Demo
Please visit the [project page](https://www.huaweicloud.com/ascend/apps/applicationDetails/167900126).
