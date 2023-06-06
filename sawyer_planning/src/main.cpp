/* Action Planning and Execution

Reference: 
1. https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
2.https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pick_place/src/pick_place_tutorial.cpp#L141

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing M.Eng. in Robotics,
University of Maryland, College Park
*/

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <chrono> 
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>
std::string action = "none";

// Function Prototypes
bool pick(moveit::planning_interface::MoveGroupInterface&, moveit::planning_interface::MoveGroupInterface::Plan&, ros::Publisher&, ros::Publisher&);
bool pour(moveit::planning_interface::MoveGroupInterface&, moveit::planning_interface::MoveGroupInterface::Plan&, ros::Publisher&, ros::Publisher&);
bool place(moveit::planning_interface::MoveGroupInterface&, moveit::planning_interface::MoveGroupInterface::Plan&, ros::Publisher&, ros::Publisher&);
void sub_callback(const std_msgs::String::ConstPtr&);
void gripper_open(ros::Publisher&, ros::Publisher&);
void gripper_close(ros::Publisher&, ros::Publisher&);

int main(int argc, char **argv){
  // Record start time
  auto start = std::chrono::high_resolution_clock::now();
  // Initialize ROS node and handle
  // Give joint_states:=/robot/joint_states as argument in command line when running this 
  ros::init(argc, argv, "sawyer_planning_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::Subscriber sub = node_handle.subscribe("action", 1000, sub_callback);
  spinner.start();
  
  ROS_INFO("ROS node initialized ... !");
 
  // Ros Publisher to control gripper
  ros::Publisher gripperR = node_handle.advertise<std_msgs::Float64>("/robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command", 1);
  ros::Publisher gripperL = node_handle.advertise<std_msgs::Float64>("/robot/electric_gripper_controller/joints/right_gripper_l_finger_controller/command", 1);
  
  
  // Set the planning group here for the group you want to plan for
  static const std::string PLANNING_GROUP = "right_arm";
  // Setup MoveGroup class using the Planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // Setup planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
   // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  // Set the planner you want to use
  move_group.setPlannerId("RRTConnectConfigDefault");
  //move_group.setPlanningTime(5.0);
  //move_group.setNumPlanningAttempts(4);
  //auto planner_params = std::map<std::string, std::string>{ {"optimization_objective", "MechanicalWorkOptimizationObjective"} };
  //move_group.setPlannerParams(move_group.getPlannerId(), "move_group", planner_params);
  
  //move_group.setRange(0.05);
  // Set position and orientation tolerances
  move_group.setGoalPositionTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.001);
  //move_group.setGoalTolerance(0.01);
  // Add collision objects
  //addCollisionObjects(planning_scene_interface);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //std::cout << "The planner params are: " << move_group.getPlannerParams() << std::endl;
  ROS_INFO("Moveit initialized ...");
  bool picked = false, poured = false, placed = false;
  ros::Rate rate(10.0);
  while(1){
    if(action == "pick" && picked == false){
      ROS_INFO("Action pick recieved!");
      picked = pick(move_group, my_plan, gripperL, gripperR);
      action = "none";
    }else if(action == "pour" && poured == false){
      ROS_INFO("Action pour recieved!");
      poured = pour(move_group, my_plan, gripperL, gripperR);
      action = "none";
    }else if(action == "place" &&  placed == false){
      ROS_INFO("Action place recieved!");
      placed = place(move_group, my_plan, gripperL, gripperR);
      action = "none";
    }else 
      continue;
    ros::spinOnce(); 
    rate.sleep();

  }
  
  // Record end time
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Execution time: " << elapsed.count() << " s\n";
  ros::Duration(0.5).sleep();
  
  
  ROS_INFO("Planning ended. Waiting for shutdown ... Ctrl+C");
  
  ros::waitForShutdown();
  //ros::shutdown();
  //ros::spin();
  return 0;
  
}

bool pick(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan, ros::Publisher &gripperL, ros::Publisher &gripperR){
  bool success;
  std::cout << "The current pose is:" << move_group.getCurrentPose().pose << std::endl;
  ros::Duration(1.0).sleep();
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p1;
  p1.orientation.w = 0.472527;
  p1.orientation.x = -0.513829;
  p1.orientation.y = -0.5175;
  p1.orientation.z = -0.494866;
  p1.position.x = -0.0236007;
  p1.position.y = 0.906033;
  p1.position.z = 0.168587;
  move_group.setPoseTarget(p1);
  
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 1 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
  //std::cout << "The current pose is:" << move_group.getCurrentPose().pose << std::endl;
  //ros::Rate r(10); // 10 hz
  gripper_open(gripperL, gripperR);
  //ros::spinOnce();
  //r.sleep();
  /*
  //pick(move_group);
   // Set the planner you want to use
  move_group.setPlannerId("RRTstarkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(4);
  // Set position and orientation tolerances
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.005);
  */
  ros::Duration(0.5).sleep();
  move_group.setStartState(*move_group.getCurrentState());
  
  geometry_msgs::Pose p2;
  p2.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  p2.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  p2.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  p2.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  p2.position.x = move_group.getCurrentPose().pose.position.x;
  p2.position.y = move_group.getCurrentPose().pose.position.y;
  p2.position.z = move_group.getCurrentPose().pose.position.z - 0.25;
  move_group.setPoseTarget(p2);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 2 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
  
 

  ros::Duration(1.0).sleep();
  
  
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p3;
  p3.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  p3.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  p3.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  p3.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  p3.position.x = move_group.getCurrentPose().pose.position.x;
  p3.position.y = move_group.getCurrentPose().pose.position.y + 0.10;
  p3.position.z = move_group.getCurrentPose().pose.position.z ;
  move_group.setPoseTarget(p3);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 3 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
  

  ros::Duration(0.5).sleep();
  gripper_close(gripperL, gripperR);
  ros::Duration(1.0).sleep();
   
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p4;
  p4.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  p4.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  p4.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  p4.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  p4.position.x = move_group.getCurrentPose().pose.position.x;
  p4.position.y = move_group.getCurrentPose().pose.position.y;
  p4.position.z = move_group.getCurrentPose().pose.position.z + 0.1;
  move_group.setPoseTarget(p4);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 4 %s", success ? "succeeded!" : "failed :(");
  move_group.move();

  ros::Duration(0.5).sleep();
  
  return true;
}

bool pour(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan, ros::Publisher &gripperL, ros::Publisher &gripperR){
  bool success;
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p5;
  p5.orientation.w = 0.685469;
  p5.orientation.x = -0.727986;
  p5.orientation.y = -0.00430406;
  p5.orientation.z = -0.0122884;
  p5.position.x = move_group.getCurrentPose().pose.position.x;
  p5.position.y = move_group.getCurrentPose().pose.position.y;
  p5.position.z = move_group.getCurrentPose().pose.position.z;
  move_group.setPoseTarget(p5);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 5 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
 
  
  ros::Duration(0.5).sleep();
  
   
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p6;
  p6.orientation.w = 0.472527;
  p6.orientation.x = -0.513829;
  p6.orientation.y = -0.5175;
  p6.orientation.z = -0.494866;
  p6.position.x = move_group.getCurrentPose().pose.position.x;
  p6.position.y = move_group.getCurrentPose().pose.position.y;
  p6.position.z = move_group.getCurrentPose().pose.position.z;
  move_group.setPoseTarget(p6);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 6 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
  ros::Duration(0.5).sleep();
  return true;
}

bool place(moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan, ros::Publisher &gripperL, ros::Publisher &gripperR){
  bool success;
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p7;
  p7.orientation.w = 0.00641904;
  p7.orientation.x = 0.700008;
  p7.orientation.y = 0.0426121;
  p7.orientation.z = 0.712834;
  p7.position.x = 0.681101;
  p7.position.y = -0.191847;
  p7.position.z = 0.154213;
  move_group.setPoseTarget(p7);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 7 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
     
  /*
     // Set the planner you want to use
  move_group.setPlannerId("RRTstarkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(4);
  // Set position and orientation tolerances
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(0.005);
  */
  ros::Duration(0.5).sleep();
  
  
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose p8;
  p8.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  p8.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  p8.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  p8.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  p8.position.x = move_group.getCurrentPose().pose.position.x;
  p8.position.y = move_group.getCurrentPose().pose.position.y;
  p8.position.z = move_group.getCurrentPose().pose.position.z - 0.23;
  move_group.setPoseTarget(p8);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan 8 %s", success ? "succeeded!" : "failed :(");
  move_group.move();
  gripper_open(gripperL, gripperR);
  return true;
}

void sub_callback(const std_msgs::String::ConstPtr& msg){
  if(msg->data == "pick"){
    action = "pick";
  }else if(msg->data == "pour"){
    action = "pour";
  }else if(msg->data == "place"){
    action = "place";
  }
}

void gripper_open(ros::Publisher &gripperL, ros::Publisher &gripperR){
  std_msgs::Float64 grip_statusL;
  std_msgs::Float64 grip_statusR;
  grip_statusR.data = -0.14;
  grip_statusL.data = 0.04;
  gripperR.publish(grip_statusR);
  gripperL.publish(grip_statusL);
}

void gripper_close(ros::Publisher &gripperL, ros::Publisher &gripperR){
  std_msgs::Float64 grip_statusL;
  std_msgs::Float64 grip_statusR;
  grip_statusR.data = 0.00;
  grip_statusL.data = 0.00;
  gripperR.publish(grip_statusR);
  gripperL.publish(grip_statusL);
}

