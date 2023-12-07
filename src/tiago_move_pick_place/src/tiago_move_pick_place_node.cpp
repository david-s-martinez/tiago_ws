#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>
#include <map>

#include <tf/transform_broadcaster.h>
#include <tiago_move_pick_place/controller.h>

namespace tiago_move {
// Creates a typedef for a SimpleActionClient that communicates with actions that adhere to the MoveBaseAction action interface.
// typedef /*#>>>>TODO: ACTION CLIENT TYPE*/</*#>>>>TODO: ACTION NAME*/> MoveBaseClient;
  Controller::Controller() : ac("move_base", true),body_planner_("arm_torso") {
  }

  Controller::~Controller()
  {
  }

  bool Controller::initialize(ros::NodeHandle& nh)
  {
    /*#>>>>TODO:Exercise3 wait for the action server to connet*/
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    /*#>>>>TODO:Exercise3 Load the 3D coordinates of three waypoints from the ROS parameter server*/
    /*#>>>>TODO:Exercise3 Store three waypoints in the vector nav_goals*/
    std::vector<std::string> waypoints = {"waypoint_A", "waypoint_A_1", "waypoint_B", "waypoint_B_1","waypoint_C", "waypoint_C_1","waypoint_C_2"};

    for (const auto &waypoint : waypoints)
    {
        move_base_msgs::MoveBaseGoal goal;

        // Retrieve the waypoint from the parameter server
        std::vector<double> waypoint_values;

        if (!ros::param::get(waypoint, waypoint_values))
        {
          ROS_ERROR("Failed to retrieve waypoint %s from the parameter server", waypoint.c_str());
          return -1;
        }

        // Fill in the MoveBaseGoal message
        goal.target_pose.header.frame_id = "map";  // Assuming waypoints are specified in the map frame
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoint_values[0];
        goal.target_pose.pose.position.y = waypoint_values[1];
        goal.target_pose.pose.orientation.w = waypoint_values[2];

        // Add the goal to the vector
        nav_goals.push_back(goal);
    }
    //#>>>>TODO:Exercise4 Load the target pose from parameter
    if(!ros::param::get("/target_pose"/*#>>>>TODO: PARAMETER NAME*/, target_pose)){
      return false; 
    }
    if(!ros::param::get("/target_pose_1"/*#>>>>TODO: PARAMETER NAME*/, target_pose_1)){
      return false; 
    }
    //#>>>>TODO:Exercise4 Set the planner of your MoveGroupInterface
    body_planner_.setPlannerId("SBLkConfigDefault");
    //#>>>>TODO:Exercise4 Set the reference frame of your target pose 
    body_planner_.setPoseReferenceFrame("base_footprint");
    return true;
  }

  move_base_msgs::MoveBaseGoal Controller::createGoal(std::vector<double> &goal)
  {
      move_base_msgs::MoveBaseGoal goal_msg;

      goal_msg.target_pose.header.frame_id = "map";//#>>>>TODO: the reference frame name
      goal_msg.target_pose.header.stamp = ros::Time::now();
      goal_msg.target_pose.pose.position.x = goal[0];
      goal_msg.target_pose.pose.position.y = goal[1];
      goal_msg.target_pose.pose.orientation.w = goal[2];

      return goal_msg;
  }

  // Uncomment the function for Exercise 4
  int Controller::move_arm(std::vector<double>& goal) {
    // input: std::vector<double> &goal [x,y,z,r,p,y]
    //#>>>>TODO:Exercise4 Create a msg of type geometry_msgs::PoseStamped from the input vector 
    //#>>>>TODO:Exercise4 Set the target pose for the planner setPoseTarget function of your MoveGroupInterface instance
    
    // Create a msg of type geometry_msgs::PoseStamped from the input vector
    geometry_msgs::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "base_footprint";  // Assuming the target pose is specified in the map frame
    target_pose_msg.header.stamp = ros::Time::now();
    target_pose_msg.pose.position.x = goal[0];
    target_pose_msg.pose.position.y = goal[1];
    target_pose_msg.pose.position.z = goal[2];
    target_pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(goal[3],goal[4],goal[5]);

    // Set the target pose for the planner setPoseTarget function of your MoveGroupInterface instance
    body_planner_.setPoseTarget(target_pose_msg);

    ROS_INFO_STREAM("Planning to move " << 
                    body_planner_.getEndEffectorLink() << " to a target pose expressed in " <<
                    body_planner_.getPlanningFrame());

    body_planner_.setStartStateToCurrentState();
    body_planner_.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    // Set maximum time to find a plan
    body_planner_.setPlanningTime(5.0);

    //#>>>>TODO:Exercise4 Start the planning by calling member function "plan" and pass the motion_plan as argument
    // Start the planning by calling member function "plan" and pass the motion_plan as an argument
    bool success = (body_planner_.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //#>>>>TODO:Exercise4 Execute the plan by calling member function "move" of the MoveGroupInterface instance
    if (success) {
        ROS_INFO_STREAM("Plan found in " << motion_plan.planning_time_ << " seconds");
        // Execute the plan by calling member function "move" of the MoveGroupInterface instance
        body_planner_.move();
    } else {
        ROS_ERROR("Failed to plan motion");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
}

// int main(int argc, char** argv){

//   ros::init(argc, argv, "tiago_move");
//   ros::NodeHandle nh;
      
//   ros::AsyncSpinner spinner(4);
//   spinner.start();

//   tiago_move::Controller controller;
//   if(!controller.initialize(nh))
//   {
//     ROS_ERROR_STREAM("tiago_move::Controller failed to initialize");
//     return -1;
//   }

//   int goal_index = 0;
//   while (ros::ok())
//   {
//       ROS_INFO("Sending goal %d", goal_index + 1);

//       //#>>>>TODO:Exercise3 send the current goal to the action server with sendGoal function of SimpleActionClient instace.
//       //#>>>>TODO:Exercise3 blocks until this goal finishes with waitForResult function of SimpleActionClient instace.
//       std::vector<double> goal_values = {
//             controller.nav_goals[goal_index].target_pose.pose.position.x,
//             controller.nav_goals[goal_index].target_pose.pose.position.y,
//             controller.nav_goals[goal_index].target_pose.pose.orientation.w
//         };

//         // Access the SimpleActionClient through the controller object
//       controller.ac.sendGoalAndWait(controller.createGoal(goal_values));
//       // Get the goal state using the controller object
//       actionlib::SimpleClientGoalState goal_state = controller.ac.getState();
//       /*#>>>>TODO:Exercise3 Check if the state of this goal is SUCCEEDED use getState function of SimpleActionClient instace*/
//       //#>>>> Hint: see https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib\_1\_1SimpleActionClient.html for the return type of getState function
//       //#>>>>TODO:Exercise4 Call the move_arm function at propoer waypoint
//       if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
//       {
//           ROS_INFO("Reached goal %d", goal_index + 1);
//           goal_index = (goal_index + 1) % controller.nav_goals.size();
//           // Check if this is the 4th goal and execute move_arm
//           if (goal_index == 3) {
//               ROS_INFO("Executing move_arm to pre lift");
//               controller.move_arm(controller.target_pose);
//               ros::Duration(1.0).sleep();
//               controller.move_arm(controller.target_pose_1);
//               ros::Duration(1.0).sleep();
//               controller.move_arm(controller.target_pose);
//               ros::Duration(1.0).sleep();
//           }
//          if (goal_index == 4) {
//               ROS_INFO("Executing move_arm to lift");
//               controller.move_arm(controller.target_pose_1);
//               ros::Duration(1.0).sleep();
//               ROS_INFO("Executing move_arm to post lift");
//               controller.move_arm(controller.target_pose);
//           }
//       }
//       else
//       {
//           ROS_WARN("Failed to reach goal %d", goal_index + 1);
//           ros::Duration(1.0).sleep();
//       }
//   }
//   spinner.stop();
//   return 0;
// }
// Alternative main to debug arm movement:

int main(int argc, char** argv){
  ros::init(argc, argv, "tiago_move");
  ros::NodeHandle nh;
      
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tiago_move::Controller controller;

  if(!controller.initialize(nh))
  {
      ROS_ERROR_STREAM("tiago_move::Controller failed to initialize");
      return -1;
  }

  int goal_index = 0;
  while (ros::ok())
  {

    ROS_INFO("Executing move_arm after reaching the 4th goal");
    // controller.move_arm(controller.target_pose);
    // ros::Duration(1.0).sleep();
    controller.move_arm(controller.target_pose_1);
    ros::Duration(1.0).sleep();
  }
  spinner.stop();
  return 0;
}
