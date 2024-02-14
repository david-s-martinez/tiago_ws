# TIAGo Robot Installation Guide and Task Execution Instructions

This document outlines the steps for setting up the TIAGo robot environment, including the installation of Ubuntu and ROS, the necessary hardware setup, and executing tasks for RoboCup@Home final task - Carry My Luggage.

## Installation

1. **Ubuntu and ROS Installation**

    Follow the instructions on the ROS wiki to install Ubuntu and ROS for TIAGo:
    [Install Ubuntu and ROS](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)

    Use the following command to integrate TIAGo's public ROS workspace:
    ```bash
    rosinstall ./src /opt/ros/noetic tiago_public−noetic.rosinstall
    ```

2. **Hardware Handbook**

    For detailed information on TIAGo's hardware, refer to the PAL Robotics handbook:
    [TIAGo Hardware Handbook](https://docs.pal-robotics.com/tiago-single/handbook.html)

3. **Web Manager**

    Access the robot's web manager at:
    [http://tiago-46c:8080/?&wtd=VtoPWd6ULoaYaohm](http://tiago-46c:8080/?&wtd=VtoPWd6ULoaYaohm)

## Prerequisite Downloads

Execute the following commands to install necessary packages and dependencies:

```bash
sudo apt install python3-pip
pip3 install deepspeech pyaudio numpy
# Download Deepspeech models and set permissions
wget https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm
wget https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer
chmod 644 /home/jin/models/models.pbmm
chmod 644 /home/jin/models/models.scorer
```

## Install PortAudio
```bash
sudo apt-get install portaudio19-dev
pip install pyaudio
```

## Install Speech Processing Libraries
```bash
pip install SpeechRecognition
pip install vosk
pip install core
pip install gtts
```

## ROS Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

# TIAGo Setup

## Connecting to TIAGo
Set Timezone to UTC
```bash
sudo timedatectl set-timezone UTC
```
To SSH into TIAGo:
```bash
ssh pal@192.168.1.200
```
To connect your computer to the robot, follow these steps:
Access the robot's web interface at [http://192.168.1.200](http://192.168.1.200)

Update your machine's IP address
ifconfig or use `ip addr` to find your IP address

Set ROS environment variables
```bash
export ROS_IP=192.168.1.<your ip addr>
export ROS_MASTER_URI=http://192.168.1.200:11311
```
Verify connectivity with the robot
```bash
ping 192.168.1.200 -c 4
```

## ROS Setup
Before working with ROS packages, source the ROS setup files:
```bash
source /opt/ros/noetic/setup.bash
```
Kill any running Gazebo instances
```bash
killall gzserver
killall gzclient
```
Source ROS Noetic setup file
```bash
source /opt/ros/noetic/setup.bash
source /devel/setup.bash
```

# RoboCup@Home Final Task Launch
Disable services using the web manager or command line
```bash
rosnode kill /head_manager
rosnode kill /map_server
rosnode kill /dock_charge_sm
```
## Launch Navigation
```bash
roslaunch robocup_at_home_final_navigation navigation.launch
```
In the web browser, manage services:
Kill move_base and localizer, then restart them

## Launch vision system
```bash
cd YOLCAT-mini-Instance-segmentation
python3 detect_tiago_bodypose.py
```
## Launch State Space Model
```bash
rosrun smach_viewer smach_viewer.py
```
## Launch MoveIt! for motion planning
```bash
roslaunch tiago_moveit_config demo.launch
```
## Launch the Project
```bash
roslaunch robocup_at_home_final_project final_task.launch
```

# Additional Notes

## Remote Control the Robot
To remote control of the robot use
```bash
rosrun key_teleop key_teleop.py
```
Note: The robot will not move if this script is running

## Test bag grasping
```bash
roslaunch tiago_moveit_config demo.launch
rosrun robocup_at_home_final_project tf_grasp.py
```

## Reset simulation environment if needed
```bash
killall gzserver
killall gzclient
rqt_graph
```

## Control arm movement
```bash
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
points:
- positions: [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}"
```

## Use with the MoveIt! command to practice grasping
```bash
rosrun robocup_at_home_final_project try_moveit.py
```
Use rosrun to give the point, try to get the point from RViz.

# TIAGo Robot Tutorials (Jin, David)

This repository contains a series of tutorials designed to guide users through various aspects of working with the TIAGo robot in ROS (Robot Operating System). Below are instructions for installing TIAGo, running simulations, performing object detection, navigation, perception, and robot manipulation.

## Installation

To install TIAGo and ROS, follow the instructions provided in the [TIAGo installation tutorial](https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS). Once installed, use the following command to set up the required packages:

```bash
rosinstall ./src /opt/ros/noetic tiago_public−noetic.rosinstall
```

## Tutorial 2: Simulation Scenario

### 1. Simulation Launch
Launch TIAGo in a simulated environment using Gazebo:

```bash
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2
```

### 2. Rviz Configuration
Launch Rviz with the TIAGo robot model:

```bash
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial
```

### 3. Controller Plugin
Run the look_to_point node for controlling the robot's gaze:

```bash
rosrun look_to_point look_to_point
```

## Tutorial 3: Mapping, Localization, and Navigation

### 1. Mapping
Generate maps for different environments using TIAGo:

```bash
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true world:=tutorial_office
```

### 2. Localization
Initialize localization for the robot:

```bash
roslaunch tiago_localization tiago_localization.launch
```

### 3. Navigation
Navigate through the mapped environment using the MoveIt! package:

```bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true
roslaunch tiago_move tiago_move.launch
```

## Tutorial 4: Object Detection and Point Cloud Segmentation

### 1. World Creation
Create a world for perception tasks:

```bash
roslaunch object_detection_world tiago.launch world_suffix:=objects
```

### 2. Object Detection
Follow the installation instructions for the Yolact repository and run object detection scripts:

```bash
cd YOLACT-mini-Instance-segmentation
python detect_tiago.py
```

## Tutorial 5: Navigation and Robot Manipulation

### 1. Navigation
Set up navigation in a custom world:

```bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=pick_place map:=$HOME/.pal/tiago_maps/configurations/pick_place_map
```

### 2. Perception
Perform perception tasks in the environment.

### 3. Robot Manipulation
Launch manipulation capabilities for TIAGo:

```bash
roslaunch tiago_move_pick_place tiago_move.launch
```

For a state machine approach, restart the setup process.

### 4. State Machine

```bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=pick_place map:=$HOME/.pal/tiago_maps/configurations/pick_place_map
sudo apt-get install ros-noetic-smach-viewer
rosrun smach_viewer smach_viewer.py
roslaunch hsrb_task_manager task_manager.launch
```

By following these tutorials, users can gain practical experience in setting up and utilizing the TIAGo robot for various robotics applications in ROS.