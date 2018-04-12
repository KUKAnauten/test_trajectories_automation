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
 *   * Neither the name of the author nor the names of its
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

/* Author: Niklas Schaefer */

#include <ros/ros.h>
#include <iimoveit/robot_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <test_trajectories_automation/MoveToJointPositionAction.h>
#include <test_trajectories_automation/PublishTrajectoryAction.h>
#include <std_msgs/UInt32.h>

// better to use <array>? -> size()
// arrays for looping through the different initial joint positions and trajectories
trajectory_msgs::JointTrajectory initial_joint_positions[3][5];
std::string trajectory_files[6];
// status * 1000000 + pose * 10000 + position * 100 + trajectory
std_msgs::UInt32 status_word;

int calc_status_word (int status, int pose, int position, int trajectory) {
  return 1000000*status + 10000*(pose+1) + 100*(position+1) + (trajectory+1);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_solver");

	ros::NodeHandle node_handle;

  ros::Publisher status_pub = node_handle.advertise<std_msgs::UInt32>("status_word", 10);

  // initial pose 1
  // ellbow central
//	trajectory_msgs::JointTrajectory initial_joint_positions11;
//	initial_joint_positions11.joint_names.resize(7);
//	initial_joint_positions11.joint_names[0] = "iiwa_joint_1";
//	initial_joint_positions11.joint_names[1] = "iiwa_joint_2";
//	initial_joint_positions11.joint_names[2] = "iiwa_joint_3";
//	initial_joint_positions11.joint_names[3] = "iiwa_joint_4";
//	initial_joint_positions11.joint_names[4] = "iiwa_joint_5"; 
//	initial_joint_positions11.joint_names[5] = "iiwa_joint_6"; 
//	initial_joint_positions11.joint_names[6] = "iiwa_joint_7";
//	initial_joint_positions11.points.resize(1);
//	initial_joint_positions11.points[0].positions.resize(7);
//	initial_joint_positions11.points[0].positions[0] = 3.1416/180.0 * -50.21;
//	initial_joint_positions11.points[0].positions[1] = 3.1416/180.0 * 31.60;
//	initial_joint_positions11.points[0].positions[2] = 3.1416/180.0 * 44.26;
//	initial_joint_positions11.points[0].positions[3] = 3.1416/180.0 * -83.30;
//	initial_joint_positions11.points[0].positions[4] = 3.1416/180.0 * -19.60; 
//	initial_joint_positions11.points[0].positions[5] = 3.1416/180.0 * -5.25; 
//	initial_joint_positions11.points[0].positions[6] = 3.1416/180.0 * 1.01;

	trajectory_msgs::JointTrajectory initial_joint_positions11;
	initial_joint_positions11.joint_names.resize(7);
	initial_joint_positions11.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions11.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions11.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions11.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions11.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions11.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions11.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions11.points.resize(1);
	initial_joint_positions11.points[0].positions.resize(7);
	initial_joint_positions11.points[0].positions[0] = 3.1416/180.0 * -30.97;
	initial_joint_positions11.points[0].positions[1] = 3.1416/180.0 * (18.34 - 90.0);
	initial_joint_positions11.points[0].positions[2] = 3.1416/180.0 * -1.0 * -21.67;
	initial_joint_positions11.points[0].positions[3] = 3.1416/180.0 * -57.57;
	initial_joint_positions11.points[0].positions[4] = 3.1416/180.0 * (-1.0 * 59.36 + 90.0); 
	initial_joint_positions11.points[0].positions[5] = 3.1416/180.0 * -4.63; 
	initial_joint_positions11.points[0].positions[6] = 3.1416/180.0 * 0.0;

  // ellbow halfway right
	trajectory_msgs::JointTrajectory initial_joint_positions12;
	initial_joint_positions12.joint_names.resize(7);
	initial_joint_positions12.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions12.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions12.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions12.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions12.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions12.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions12.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions12.points.resize(1);
	initial_joint_positions12.points[0].positions.resize(7);
	initial_joint_positions12.points[0].positions[0] = 3.1416/180.0 * -60.16;
	initial_joint_positions12.points[0].positions[1] = 3.1416/180.0 * 39.65;
	initial_joint_positions12.points[0].positions[2] = 3.1416/180.0 * 64.05;
	initial_joint_positions12.points[0].positions[3] = 3.1416/180.0 * -83.30;
	initial_joint_positions12.points[0].positions[4] = 3.1416/180.0 * 54.02; 
	initial_joint_positions12.points[0].positions[5] = 3.1416/180.0 * -9.43; 
	initial_joint_positions12.points[0].positions[6] = 3.1416/180.0 * -84.28;

  // ellbow right
	trajectory_msgs::JointTrajectory initial_joint_positions13;
	initial_joint_positions13.joint_names.resize(7);
	initial_joint_positions13.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions13.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions13.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions13.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions13.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions13.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions13.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions13.points.resize(1);
	initial_joint_positions13.points[0].positions.resize(7);
	initial_joint_positions13.points[0].positions[0] = 3.1416/180.0 * -65.25;
	initial_joint_positions13.points[0].positions[1] = 3.1416/180.0 * 64.14;
	initial_joint_positions13.points[0].positions[2] = 3.1416/180.0 * 97.21;
	initial_joint_positions13.points[0].positions[3] = 3.1416/180.0 * -83.30;
	initial_joint_positions13.points[0].positions[4] = 3.1416/180.0 * 62.11; 
	initial_joint_positions13.points[0].positions[5] = 3.1416/180.0 * -32.83; 
	initial_joint_positions13.points[0].positions[6] = 3.1416/180.0 * -122.93;

  // ellbow halfway left
	trajectory_msgs::JointTrajectory initial_joint_positions14;
	initial_joint_positions14.joint_names.resize(7);
	initial_joint_positions14.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions14.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions14.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions14.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions14.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions14.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions14.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions14.points.resize(1);
	initial_joint_positions14.points[0].positions.resize(7);
	initial_joint_positions14.points[0].positions[0] = 3.1416/180.0 * -27.46;
	initial_joint_positions14.points[0].positions[1] = 3.1416/180.0 * 25.94;
	initial_joint_positions14.points[0].positions[2] = 3.1416/180.0 * 9.78;
	initial_joint_positions14.points[0].positions[3] = 3.1416/180.0 * -83.30;
	initial_joint_positions14.points[0].positions[4] = 3.1416/180.0 * -62.71; 
	initial_joint_positions14.points[0].positions[5] = 3.1416/180.0 * -14.89; 
	initial_joint_positions14.points[0].positions[6] = 3.1416/180.0 * 58.61;

  // ellbow left
	trajectory_msgs::JointTrajectory initial_joint_positions15;
	initial_joint_positions15.joint_names.resize(7);
	initial_joint_positions15.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions15.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions15.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions15.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions15.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions15.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions15.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions15.points.resize(1);
	initial_joint_positions15.points[0].positions.resize(7);
	initial_joint_positions15.points[0].positions[0] = 3.1416/180.0 * 24.60;
	initial_joint_positions15.points[0].positions[1] = 3.1416/180.0 * 58.94;
	initial_joint_positions15.points[0].positions[2] = 3.1416/180.0 * -91.29;
	initial_joint_positions15.points[0].positions[3] = 3.1416/180.0 * -83.30;
	initial_joint_positions15.points[0].positions[4] = 3.1416/180.0 * -46.37; 
	initial_joint_positions15.points[0].positions[5] = 3.1416/180.0 * -57.73; 
	initial_joint_positions15.points[0].positions[6] = 3.1416/180.0 * 106.04;


  // initial pose 2
  // ellbow central
	trajectory_msgs::JointTrajectory initial_joint_positions21;
	initial_joint_positions21.joint_names.resize(7);
	initial_joint_positions21.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions21.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions21.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions21.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions21.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions21.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions21.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions21.points.resize(1);
	initial_joint_positions21.points[0].positions.resize(7);
	initial_joint_positions21.points[0].positions[0] = 3.1416/180.0 * 19.66;
	initial_joint_positions21.points[0].positions[1] = 3.1416/180.0 * 26.32;
	initial_joint_positions21.points[0].positions[2] = 3.1416/180.0 * -23.71;
	initial_joint_positions21.points[0].positions[3] = 3.1416/180.0 * -71.01;
	initial_joint_positions21.points[0].positions[4] = 3.1416/180.0 * 10.16; 
	initial_joint_positions21.points[0].positions[5] = 3.1416/180.0 * 84.74; 
	initial_joint_positions21.points[0].positions[6] = 3.1416/180.0 * -92.03;

  // ellbow halfway right
	trajectory_msgs::JointTrajectory initial_joint_positions22;
	initial_joint_positions22.joint_names.resize(7);
	initial_joint_positions22.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions22.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions22.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions22.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions22.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions22.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions22.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions22.points.resize(1);
	initial_joint_positions22.points[0].positions.resize(7);
	initial_joint_positions22.points[0].positions[0] = 3.1416/180.0 * -33.60;
	initial_joint_positions22.points[0].positions[1] = 3.1416/180.0 * 39.38;
	initial_joint_positions22.points[0].positions[2] = 3.1416/180.0 * 69.06;
	initial_joint_positions22.points[0].positions[3] = 3.1416/180.0 * -71.02;
	initial_joint_positions22.points[0].positions[4] = 3.1416/180.0 * -36.47; 
	initial_joint_positions22.points[0].positions[5] = 3.1416/180.0 * 92.23; 
	initial_joint_positions22.points[0].positions[6] = 3.1416/180.0 * -60.77;

  // ellbow right
	trajectory_msgs::JointTrajectory initial_joint_positions23;
	initial_joint_positions23.joint_names.resize(7);
	initial_joint_positions23.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions23.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions23.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions23.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions23.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions23.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions23.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions23.points.resize(1);
	initial_joint_positions23.points[0].positions.resize(7);
	initial_joint_positions23.points[0].positions[0] = 3.1416/180.0 * -35.98;
	initial_joint_positions23.points[0].positions[1] = 3.1416/180.0 * 59.46;
	initial_joint_positions23.points[0].positions[2] = 3.1416/180.0 * 100.62;
	initial_joint_positions23.points[0].positions[3] = 3.1416/180.0 * -71.01;
	initial_joint_positions23.points[0].positions[4] = 3.1416/180.0 * -63.23; 
	initial_joint_positions23.points[0].positions[5] = 3.1416/180.0 * 108.52; 
	initial_joint_positions23.points[0].positions[6] = 3.1416/180.0 * -46.91;

  // ellbow halfway left
	trajectory_msgs::JointTrajectory initial_joint_positions24;
	initial_joint_positions24.joint_names.resize(7);
	initial_joint_positions24.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions24.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions24.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions24.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions24.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions24.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions24.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions24.points.resize(1);
	initial_joint_positions24.points[0].positions.resize(7);
	initial_joint_positions24.points[0].positions[0] = 3.1416/180.0 * 42.14;
	initial_joint_positions24.points[0].positions[1] = 3.1416/180.0 * 39.24;
	initial_joint_positions24.points[0].positions[2] = 3.1416/180.0 * -68.80;
	initial_joint_positions24.points[0].positions[3] = 3.1416/180.0 * -71.01;
	initial_joint_positions24.points[0].positions[4] = 3.1416/180.0 * 36.01; 
	initial_joint_positions24.points[0].positions[5] = 3.1416/180.0 * 92.03; 
	initial_joint_positions24.points[0].positions[6] = 3.1416/180.0 * -109.00;

  // ellbow left
	trajectory_msgs::JointTrajectory initial_joint_positions25;
	initial_joint_positions25.joint_names.resize(7);
	initial_joint_positions25.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions25.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions25.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions25.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions25.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions25.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions25.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions25.points.resize(1);
	initial_joint_positions25.points[0].positions.resize(7);
	initial_joint_positions25.points[0].positions[0] = 3.1416/180.0 * 44.57;
	initial_joint_positions25.points[0].positions[1] = 3.1416/180.0 * 59.61;
	initial_joint_positions25.points[0].positions[2] = 3.1416/180.0 * -100.81;
	initial_joint_positions25.points[0].positions[3] = 3.1416/180.0 * -71.01;
	initial_joint_positions25.points[0].positions[4] = 3.1416/180.0 * 63.17; 
	initial_joint_positions25.points[0].positions[5] = 3.1416/180.0 * 108.48; 
	initial_joint_positions25.points[0].positions[6] = 3.1416/180.0 * -123.11;
  

  // initial pose 3
  // ellbow central
	trajectory_msgs::JointTrajectory initial_joint_positions31;
	initial_joint_positions31.joint_names.resize(7);
	initial_joint_positions31.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions31.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions31.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions31.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions31.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions31.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions31.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions31.points.resize(1);
	initial_joint_positions31.points[0].positions.resize(7);
	initial_joint_positions31.points[0].positions[0] = 3.1416/180.0 * 22.06;
	initial_joint_positions31.points[0].positions[1] = 3.1416/180.0 * 7.84;
	initial_joint_positions31.points[0].positions[2] = 3.1416/180.0 * -20.02;
	initial_joint_positions31.points[0].positions[3] = 3.1416/180.0 * -88.09;
	initial_joint_positions31.points[0].positions[4] = 3.1416/180.0 * -28.66; 
	initial_joint_positions31.points[0].positions[5] = 3.1416/180.0 * 16.52; 
	initial_joint_positions31.points[0].positions[6] = 3.1416/180.0 * -21.04;

  // ellbow halfway right
	trajectory_msgs::JointTrajectory initial_joint_positions32;
	initial_joint_positions32.joint_names.resize(7);
	initial_joint_positions32.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions32.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions32.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions32.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions32.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions32.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions32.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions32.points.resize(1);
	initial_joint_positions32.points[0].positions.resize(7);
	initial_joint_positions32.points[0].positions[0] = 3.1416/180.0 * -56.37;
	initial_joint_positions32.points[0].positions[1] = 3.1416/180.0 * 38.40;
	initial_joint_positions32.points[0].positions[2] = 3.1416/180.0 * 98.89;
	initial_joint_positions32.points[0].positions[3] = 3.1416/180.0 * -88.09;
	initial_joint_positions32.points[0].positions[4] = 3.1416/180.0 * -97.27; 
	initial_joint_positions32.points[0].positions[5] = 3.1416/180.0 * 52.89; 
	initial_joint_positions32.points[0].positions[6] = 3.1416/180.0 * 10.21;

  // ellbow right
	trajectory_msgs::JointTrajectory initial_joint_positions33;
	initial_joint_positions33.joint_names.resize(7);
	initial_joint_positions33.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions33.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions33.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions33.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions33.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions33.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions33.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions33.points.resize(1);
	initial_joint_positions33.points[0].positions.resize(7);
	initial_joint_positions33.points[0].positions[0] = 3.1416/180.0 * -34.68;
	initial_joint_positions33.points[0].positions[1] = 3.1416/180.0 * 75.73;
	initial_joint_positions33.points[0].positions[2] = 3.1416/180.0 * 134.43;
	initial_joint_positions33.points[0].positions[3] = 3.1416/180.0 * -88.09;
	initial_joint_positions33.points[0].positions[4] = 3.1416/180.0 * -136.87; 
	initial_joint_positions33.points[0].positions[5] = 3.1416/180.0 * 91.95; 
	initial_joint_positions33.points[0].positions[6] = 3.1416/180.0 * -10.11;

  // ellbow halfway left
	trajectory_msgs::JointTrajectory initial_joint_positions34;
	initial_joint_positions34.joint_names.resize(7);
	initial_joint_positions34.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions34.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions34.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions34.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions34.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions34.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions34.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions34.points.resize(1);
	initial_joint_positions34.points[0].positions.resize(7);
	initial_joint_positions34.points[0].positions[0] = 3.1416/180.0 * 66.18;
	initial_joint_positions34.points[0].positions[1] = 3.1416/180.0 * 33.76;
	initial_joint_positions34.points[0].positions[2] = 3.1416/180.0 * -94.58;
	initial_joint_positions34.points[0].positions[3] = 3.1416/180.0 * -88.09;
	initial_joint_positions34.points[0].positions[4] = 3.1416/180.0 * 72.19; 
	initial_joint_positions34.points[0].positions[5] = 3.1416/180.0 * 30.40; 
	initial_joint_positions34.points[0].positions[6] = 3.1416/180.0 * -95.49;

  // ellbow left
	trajectory_msgs::JointTrajectory initial_joint_positions35;
	initial_joint_positions35.joint_names.resize(7);
	initial_joint_positions35.joint_names[0] = "iiwa_joint_1";
	initial_joint_positions35.joint_names[1] = "iiwa_joint_2";
	initial_joint_positions35.joint_names[2] = "iiwa_joint_3";
	initial_joint_positions35.joint_names[3] = "iiwa_joint_4";
	initial_joint_positions35.joint_names[4] = "iiwa_joint_5"; 
	initial_joint_positions35.joint_names[5] = "iiwa_joint_6"; 
	initial_joint_positions35.joint_names[6] = "iiwa_joint_7";
	initial_joint_positions35.points.resize(1);
	initial_joint_positions35.points[0].positions.resize(7);
	initial_joint_positions35.points[0].positions[0] = 3.1416/180.0 * 42.69;
	initial_joint_positions35.points[0].positions[1] = 3.1416/180.0 * 76.59;
	initial_joint_positions35.points[0].positions[2] = 3.1416/180.0 * -135.52;
	initial_joint_positions35.points[0].positions[3] = 3.1416/180.0 * -88.09;
	initial_joint_positions35.points[0].positions[4] = 3.1416/180.0 * 121.23; 
	initial_joint_positions35.points[0].positions[5] = 3.1416/180.0 * 79.07; 
	initial_joint_positions35.points[0].positions[6] = 3.1416/180.0 * -88.69;


  initial_joint_positions[0][0] = initial_joint_positions11;
  initial_joint_positions[0][1] = initial_joint_positions12;
  initial_joint_positions[0][2] = initial_joint_positions13;
  initial_joint_positions[0][3] = initial_joint_positions14;
  initial_joint_positions[0][4] = initial_joint_positions15;
  initial_joint_positions[1][0] = initial_joint_positions21;
  initial_joint_positions[1][1] = initial_joint_positions22;
  initial_joint_positions[1][2] = initial_joint_positions23;
  initial_joint_positions[1][3] = initial_joint_positions24;
  initial_joint_positions[1][4] = initial_joint_positions25;
  initial_joint_positions[2][0] = initial_joint_positions31;
  initial_joint_positions[2][1] = initial_joint_positions32;
  initial_joint_positions[2][2] = initial_joint_positions33;
  initial_joint_positions[2][3] = initial_joint_positions34;
  initial_joint_positions[2][4] = initial_joint_positions35;

  // pose trajectory files 
//  trajectory_files[0] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/x_trans.csv";
//  trajectory_files[1] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/y_trans.csv";
//  trajectory_files[2] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/z_trans.csv";
//  trajectory_files[3] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/x_rot.csv";
//  trajectory_files[4] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/y_rot.csv";
//  trajectory_files[5] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/20180406_absolute_orientation/z_rot.csv";

  // joint trajectory files
//  trajectory_files[0] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_x_trans.csv";
//  trajectory_files[1] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_y_trans.csv";
//  trajectory_files[2] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_z_trans.csv";
//  trajectory_files[3] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_x_rot.csv";
//  trajectory_files[4] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_y_rot.csv";
//  trajectory_files[5] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints/20180327_basic_trajectories/mcs_joints_z_rot.csv";
  
  // joint and pose trajectory files
  trajectory_files[0] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_x_trans.csv";
  trajectory_files[1] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_y_trans.csv";
  trajectory_files[2] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_z_trans.csv";
  trajectory_files[3] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_x_rot.csv";
  trajectory_files[4] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_y_rot.csv";
  trajectory_files[5] = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/joints_and_poses/mcs_joints_and_poses_z_rot.csv"; 

  // create the action clients
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<test_trajectories_automation::MoveToJointPositionAction> ac_follow("as_pose_follower", true);
  actionlib::SimpleActionClient<test_trajectories_automation::PublishTrajectoryAction> ac_publish("as_publish_trajectory", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action servers to start.");
  ac_follow.waitForServer();
  ac_publish.waitForServer(); //will wait for infinite time
  ROS_INFO("Action servers started.");

  test_trajectories_automation::MoveToJointPositionGoal goal_follow;
	test_trajectories_automation::PublishTrajectoryGoal goal_publish;


  for (int pose_index=0; pose_index<1; pose_index++) {
    for (int position_index=0; position_index<1; position_index++) {
      for (int trajectory_index=0; trajectory_index<6; trajectory_index++) {
        // moving to initial position
        status_word.data = calc_status_word(2, pose_index, position_index, trajectory_index);
        status_pub.publish(status_word);
        goal_follow.joint_positions = initial_joint_positions[pose_index][position_index];
        ac_follow.sendGoal(goal_follow);
        ROS_INFO("Moving to pose %d in position %d.", pose_index, position_index);
        ac_follow.waitForResult();
        // publishing trajectory
        status_word.data = calc_status_word(3, pose_index, position_index, trajectory_index);
        status_pub.publish(status_word);
        goal_publish.filename = trajectory_files[trajectory_index];
	      ac_publish.sendGoal(goal_publish);
        ROS_INFO("Publishing trajectory %d.", trajectory_index);
        ac_publish.waitForResult();
      }
    }  
  }
  
  status_word.data = calc_status_word(4, 0, 0, 0);
  status_pub.publish(status_word);


//  goal_follow.joint_positions = initial_joint_positions;
//  ac_follow.sendGoal(goal_follow);
//  ROS_INFO("Goal sent. Waiting for completion.");
//  ac_follow.waitForResult();

//  ROS_INFO("Publish first trajectory.");
//	goal_publish.filename = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/safe_workspace_messung1/y-axis_320353.csv";
//	ac_publish.sendGoal(goal_publish);
//  ROS_INFO("Goal sent. Waiting for completion.");
//  ac_publish.waitForResult();

//  goal_follow.joint_positions = initial_joint_positions2;
//  ac_follow.sendGoal(goal_follow);
//  ROS_INFO("Goal sent. Waiting for completion.");
//  ac_follow.waitForResult();
//  
//  ROS_INFO("Publish second trajectory.");
//	goal_publish.filename = "/home/niklas/ROS/niklas_ws/src/test_trajectories_automation/offline_files/poses/safe_workspace_messung1/y-axis_320353.csv";
//	ac_publish.sendGoal(goal_publish);
//  ROS_INFO("Goal sent. Waiting for completion.");  
//  ac_publish.waitForResult();

  ROS_INFO("gg ez"); 
	

//  ROS_INFO("Waiting for action server to start.");
//  // wait for the action server to start
//  ac_follow.waitForServer(); //will wait for infinite time

//  ROS_INFO("Action server started, waiting for approval to send goal.");
////	pose_follower.waitForApproval();
//  // send a goal to the action
//  test_trajectories_automation::MoveToJointPositionGoal goal;
//  goal.joint_positions = initial_joint_positions;
//  ac_follow.sendGoal(goal);

//  //wait for the action to return
//  bool finished_before_timeout = ac_follow.waitForResult(ros::Duration(30.0));

//  if (finished_before_timeout)
//  {
//    actionlib::SimpleClientGoalState state = ac_follow.getState();
//    ROS_INFO("Action finished: %s",state.toString().c_str());
//		ROS_INFO("%f", ac_follow.getResult()->test);
//  }
//  else
//    ROS_INFO("Action did not finish before the time out.");

//  ROS_INFO("Waiting for 120 sec.");
//	ros::Duration(120.0).sleep(); 	
//  goal.joint_positions = initial_joint_positions2;
//  ac_follow.sendGoal(goal);

//  //wait for the action to return
//  bool finished_before_timeout2 = ac_follow.waitForResult(ros::Duration(30.0));

//  if (finished_before_timeout2)
//  {
//    actionlib::SimpleClientGoalState state = ac_follow.getState();
//    ROS_INFO("Action finished: %s",state.toString().c_str());
//		ROS_INFO("%f", ac_follow.getResult()->test);
//  }
//  else
//    ROS_INFO("Action did not finish before the time out.");

  ros::Rate rate(100);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();	
  //exit
  return 0;
}

