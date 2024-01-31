Finial task
# Tiago is using UTC, so changing the time zone on your PC to UTC zone is better.

$ sudo timedatectl set-timezone UTC

ssh pal@192.168.1.200

source /opt/ros/noetic/setup.bash

export ROS_IP=192.168.1.107
export ROS_MASTER_URI=http://192.168.1.200:11311

ping 192.168.1.200
on web: 192.168.1.200
http://tiago-46c:8080/?&wtd=VtoPWd6ULoaYaohm

killall gzserver
killall gzclient
rqt_graph

rosrun key_teleop key_teleop.py

demo
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper

roslaunch object_detection_world tiago.launch world_suffix:=empty

rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
use GUI to look objects :(controller--->head_controller head_1_joint:0 head_2_joint:-0.8)
zhege yongde shi shenme topic

sudo apt install python3-pip

pip3 install deepspeech pyaudio numpy
download deepspeech-0.9.3-models.pbmm  models.scorer
chmod 644 /home/jin/models/models.pbmm
chmod 644 /home/jin/models/models.scorer

cd ~/Downloads/stt_pratikum_python/Jin

I can resolve that problem. First ejecute:
sudo apt-get install portaudio19-dev
Next, ejecute:
pip install pyaudio

pip install SpeechRecognition
pip install vosk
pip install core
pip install gtts

1: Navigation
First move the src/object_detection_world/worlds/pick_place.world file to pal_gazebo_worlds/worlds

$ roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=pick_place map:=$HOME/.pal/tiago_maps/configurations/pick_place_map
*Change map path according to your system, all maps are found under the ./maps/ directory in this repo.

$ rosservice call /global_localization "{}"

$ roslaunch tiago_localization tiago_localization.launch

2: Perception

3: Robot manipulation
$ roslaunch tiago_move_pick_place tiago_move.launch

if you want to use state machine you should start over
4: State 

$ roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=pick_place map:=$HOME/.pal/tiago_maps/configurations/pick_place_map

$ sudo apt-get install ros-noetic-smach-viewer

$ rosrun smach_viewer smach_viewer.py

 /tiago_ws/src/hsrb_task_manager/script chmod + finial_task.py
 
$ roslaunch a_finial_project_robcup_athome finial_task.launch



[ERROR] [1706279130.464341748]: Client [/move_group] wants topic /move_group/goal to have datatype/md5sum [moveit_msgs/MoveGroupActionGoal/152e336e337dce7cbe639f1bd9c65def], but our version has [moveit_msgs/MoveGroupActionGoal/b7138704cefd43a8dd9758d697350b85]. Dropping connection.
Traceback (most recent call last):
  File "/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/scripts/try_grasp.py", line 49, in <module>
    main()
  File "/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/scripts/try_grasp.py", line 40, in main
    arm_mover = TiagoArmMover()
  File "/home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/scripts/try_grasp.py", line 17, in __init__
    move_group = moveit_commander.MoveGroupCommander(group_name)
  File "/opt/ros/noetic/lib/python3/dist-packages/moveit_commander/move_group.py", line 66, in __init__
    self._g = _moveit_move_group_interface.MoveGroupInterface(
RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)
[finial_task_manager_node-1] process has died [pid 20347, exit code 1, cmd /home/jin/ros/tiago_ws/src/a_finial_project_robcup_athome/scripts/try_grasp.py __name:=finial_task_manager_node __log:=/home/jin/.ros/log/c134e504-bc55-11ee-a34c-3413e85a2634/finial_task_manager_node-1.log].
log file: /home/jin/.ros/log/c134e504-bc55-11ee-a34c-3413e85a2634/finial_task_manager_node-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor 








rostopic echo /arm_controller/command 


header: 
  seq: 1913
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
joint_names: 
  - arm_1_joint
  - arm_2_joint
  - arm_3_joint
  - arm_4_joint
  - arm_5_joint
  - arm_6_joint
  - arm_7_joint
points: 
  - 
    positions: [0.4, -1.17, -1.9, 2.3, -1.3, -0.45, 1.75]
    accelerations: []
    effort: []
    time_from_start: 
      secs: 1
      nsecs:         0
---


rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
points:
- positions: [0.07, -0.5, -0.3, 2.13, -1.33, -0.43, -0.2]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}"

roslaunch tiago_pick_demo pick_demo.launch 
    Please remove the parameter '/robot_description_kinematics/arm/kinematics_solver_attempts' from your configuration.

roslaunch tiago_moveit_config demo.launch
rosrun a_finial_project_robcup_athome try_moveit.py 
use them together , rosrun to give the point , try to get the point from rviz.
 