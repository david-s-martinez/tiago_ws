Tiago Installation:
https://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS
rosinstall ./src /opt/ros/noetic tiago_public−noetic.rosinstall

Tutorial 2:
Tiago was used for all.

6.1 Simulation scenario:
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial2

6.2 Rviz configuration file
roslaunch ics_gazebo tiago.launch world_suffix:=tutorial

6.3 Controller plugin
rosrun look_to_point look_to_point
TODO based on look_to_point rosrun:
roslaunch controllers_tutorials combined_resource_controller_tiago.launch

Tutorial 3:
Tiago was used for all.

1: Generate maps for two worlds using Tiago 
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true world:=tutorial_office
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=$HOME/.pal/tiago_maps/configurations/tutorial3_1
*Change map path accordingly

2: Localization
roslaunch tiago_localization tiago_localization.launch

3: Navigate the Map, 4: Planning in Cartesian space with MoveIt!
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true
roslaunch tiago_move tiago_move.launch

Tutorial 4:
Tiago was used for all.
1: Create a world for perception
roslaunch object_detection_world tiago.launch world_suffix:=objects
rosrun look_to_point look_to_point

____________________________________________
Installation of the tmc ROS Noetic packages.

These are not free packages. Please install them in your remote workstation and do not distribute the instruction set. Open one terminal and run the following comands one by one.

$ sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
$ sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | sudo apt-key add -
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo sh -c 'mkdir -p /etc/apt/auth.conf.d'
$ sudo sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
$ sudo apt-get update
$ sudo apt-get install ros-noetic-tmc-desktop-full


################### Command and message to publish to the mobile base

# For Tiago

rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "linear:
 x: 1.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0" -r 3

# For the HSRB

rostopic pub /base_velocity geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 3
 
 
################### Command and message to publish to the arm joints.

# For Tiago:

rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:  
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
points: 
  - 
    positions: [0.5, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 1
      nsecs: 0"
      

# For HSRB:

$ rostopic pub /hsrb/arm_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: [arm_flex_joint, arm_lift_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint]
points:
  - 
    positions: [0, 0, 0, 0, 0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 1
      nsecs: 0"


rostopic pub /hsrb/arm_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
seq: 0
stamp:
secs: 0
nsecs: 0
frame_id: ''
joint_names: [arm_flex_joint, arm_lift_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint]
points:
- positions: [0, 0, 0, 0, 0]
velocities: []
accelerations: []
effort: []
time_from_start: {secs: 1, nsecs: 0}"


      
