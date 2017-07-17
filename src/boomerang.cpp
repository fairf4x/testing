/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "nav_msgs/Path.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/RobotState.h"
#include "moveit/robot_state/conversions.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "testing/GetRobotTrajectoryFromPath.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bebop_boomerang");
  ros::NodeHandle node_handle;

    if (argc == 2)
    {
    	sourceTopicName = argv[1];
    } 


  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "Bebop";//AndKinect

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom");
  visual_tools.deleteAllMarkers();


  ROS_INFO_NAMED("boomerang", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  visual_tools.trigger();
  visual_tools.prompt("next step");
  

  ros::Publisher statePublisher;
  statePublisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1); 

  moveit::planning_interface::MoveGroupInterface::Plan backtrackingPlan;
  moveit_msgs::DisplayTrajectory displayTrajectory;
  displayTrajectory.model_id = "bebop";

  moveit_msgs::RobotState robotStateMsg;
  auto currentState = move_group.getCurrentState();  
  moveit::core::robotStateToRobotStateMsg(*currentState.get(), robotStateMsg);



  backtrackingPlan.start_state_ = robotStateMsg;
  displayTrajectory.trajectory_start = robotStateMsg;


  testing::GetRobotTrajectoryFromPath trajectoryFromPath;
ros::ServiceClient trajectoryClient = 
          node_handle.serviceClient<testing::GetRobotTrajectoryFromPath>("get_robot_trajectory_from_path");
    ROS_INFO("Calling path to robot trajectory service.");
  ROS_INFO_NAMED("backtracker", "Service registered.");


 /* nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "odom";

  ros::Time start = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = start;
  pose.pose.orientation.w = 1;
  path.poses.push_back(pose);

  pose.header.stamp = start + ros::Duration(1);
  pose.pose.position.x = 1;
  pose.pose.position.z = 1;
  path.poses.push_back(pose);

  pose.header.stamp = start + ros::Duration(2);
  pose.pose.position.y = 1;
  path.poses.push_back(pose);*/

  trajectoryFromPath.request.path = turnPathAround();
  trajectoryFromPath.request.joint_names.push_back("Base");

	if (trajectoryClient.call(trajectoryFromPath))
        {
	    ROS_INFO("Received response.");
    	}
    	else
    	{
      	    ROS_ERROR("Failed to call service transformation.");
	    ros::shutdown();
	    return 1;
	}

  //robotTrajectoryMsg.multi_dof_joint_trajectory = multi;
  displayTrajectory.trajectory.push_back(trajectoryFromPath.response.trajectory);
  backtrackingPlan.trajectory_ = trajectoryFromPath.response.trajectory;

  statePublisher.publish(displayTrajectory);

  ROS_INFO_NAMED("tutorial", "Visualizing backtrackingPlan as a trajectory");
  visual_tools.trigger();
  visual_tools.prompt("next step");

  ROS_INFO_NAMED("tutorial", "Executing backtrackingPlan");
  move_group.execute(backtrackingPlan);

  visual_tools.trigger();
  visual_tools.prompt("next step");

  ros::shutdown();
  return 0;
}
