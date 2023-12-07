#!/usr/bin/env python3
import rospy
import actionlib
import tf
import moveit_commander
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from time import sleep

class Controller:
    def __init__(self):
        rospy.init_node('tiago_move', anonymous=True)

        # Exercise 4: Create a MoveGroupCommander for MoveGroup "arm_torso"
        self.body_planner = moveit_commander.MoveGroupCommander("arm_torso")

        # Exercise 3: Creates a SimpleActionClient that communicates with the move_base action server.
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Exercise 3: Create a list to store the waypoints
        self.nav_goals = []

        # Exercise 4: Load target poses from parameters
        self.target_pose = rospy.get_param("/target_pose")
        self.target_pose_1 = rospy.get_param("/target_pose_1")

        # Exercise 4: Set planner and reference frame
        self.body_planner.set_planner_id("SBLkConfigDefault")
        self.body_planner.set_pose_reference_frame("base_footprint")

    def create_goal(self, goal):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose.position.x = goal[0]
        goal_msg.target_pose.pose.position.y = goal[1]
        goal_msg.target_pose.pose.orientation.w = goal[2]
        return goal_msg

    # Exercise 4: Move the robot's arm
    def move_arm(self, goal):
        # Create a PoseStamped message from the input vector
        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = "base_footprint"
        target_pose_msg.header.stamp = rospy.Time.now()
        target_pose_msg.pose.position.x = goal[0]
        target_pose_msg.pose.position.y = goal[1]
        target_pose_msg.pose.position.z = goal[2]
        quaternion = tf.transformations.quaternion_from_euler(goal[3], goal[4], goal[5])
        target_pose_msg.pose.orientation.x = quaternion[0]
        target_pose_msg.pose.orientation.y = quaternion[1]
        target_pose_msg.pose.orientation.z = quaternion[2]
        target_pose_msg.pose.orientation.w = quaternion[3]

        # Set the target pose for the MoveGroupCommander
        self.body_planner.set_pose_target(target_pose_msg)

        rospy.loginfo("Planning to move {} to a target pose expressed in {}".format(
            self.body_planner.get_end_effector_link(), self.body_planner.get_planning_frame()))

        self.body_planner.set_start_state_to_current_state()
        self.body_planner.set_max_velocity_scaling_factor(1.0)

        # Create a motion plan
        motion_plan = self.body_planner.plan()
        print(len(motion_plan))
        # Set maximum time to find a plan
        self.body_planner.set_planning_time(5.0)

        # Execute the plan
        success = self.body_planner.go()

        if success:
            rospy.loginfo("Plan found in")
        else:
            rospy.logerr("Failed to plan motion")

def main():
    rospy.init_node('tiago_move', anonymous=True)
    controller = Controller()

    # Exercise 3: Wait for the move_base action server to come up
    controller.move_base_client.wait_for_server()

    waypoints = ["waypoint_A", "waypoint_A_1", "waypoint_B", "waypoint_B_1",
                 "waypoint_C", "waypoint_C_1", "waypoint_C_2"]

    for waypoint in waypoints:
        waypoint_values = rospy.get_param(waypoint)
        goal_msg = controller.create_goal(waypoint_values)
        controller.nav_goals.append(goal_msg)

    rate = rospy.Rate(1)
    goal_index = 0

    while not rospy.is_shutdown():
        rospy.loginfo("Sending goal {}".format(goal_index + 1))
        
        # Exercise 3: Send the current goal to the action server
        controller.move_base_client.send_goal(controller.nav_goals[goal_index])
        controller.move_base_client.wait_for_result()
        # Check if the state of this goal is SUCCEEDED
        goal_state = controller.move_base_client.get_state()
        print("state", goal_state)
        if goal_state == 3:
            rospy.loginfo("Reached goal {}".format(goal_index + 1))
            goal_index = (goal_index + 1) % len(controller.nav_goals)

            # Exercise 4: Call the move_arm function at proper waypoint
            if goal_index == 3:
                rospy.loginfo("Executing move_arm to pre lift")
                controller.move_arm(controller.target_pose)
                sleep(1.0)
                controller.move_arm(controller.target_pose_1)
                sleep(1.0)
                controller.move_arm(controller.target_pose)
                sleep(1.0)

            if goal_index == 4:
                rospy.loginfo("Executing move_arm to lift")
                controller.move_arm(controller.target_pose_1)
                sleep(1.0)
                rospy.loginfo("Executing move_arm to post lift")
                controller.move_arm(controller.target_pose)

        else:
            rospy.logwarn("Failed to reach goal {}".format(goal_index + 1))
            sleep(1.0)

        rate.sleep()

if __name__ == '__main__':
    
    main()
   
