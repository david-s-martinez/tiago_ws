Finial task
# Tiago is using UTC, so changing the time zone on your PC to UTC zone is better.

$ sudo timedatectl set-timezone UTC

start the robot 

ssh pal@192.168.1.200

source /opt/ros/noetic/setup.bash

rosnode kill /dock_charge_sm


export ROS_IP=192.168.1.109
export ROS_MASTER_URI=http://192.168.1.200:11311

ping 192.168.1.200
on web: 192.168.1.200
http://tiago-46c:8080/?&wtd=VtoPWd6ULoaYaohm

How to run the project
$ rosrun smach_viewer smach_viewer.py
$ roslaunch tiago_moveit_config demo.launch
$ roslaunch a_finial_project_robcup_athome finial_task.launch
to test the grasp bag
$ roslaunch tiago_moveit_config demo.launch
$ rosrun a_finial_project_robcup_athome tf_grasp.py


$ killall gzserver
$ killall gzclient
$ rqt_graph

rosrun key_teleop key_teleop.py

demo
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper

roslaunch object_detection_world tiago.launch world_suffix:=empty


move the joint
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
    use GUI to look objects :(controller--->head_controller head_1_joint:0 head_2_joint:-0.8)


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

roslaunch tiago_pick_demo pick_demo.launch 
    Please remove the parameter '/robot_description_kinematics/arm/kinematics_solver_attempts' from your configuration.


rosrun a_finial_project_robcup_athome try_moveit.py 
use them together , rosrun to give the point , try to get the point from rviz.
