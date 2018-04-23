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
#include <actionlib/server/simple_action_server.h>
#include <test_trajectories_automation/MoveToJointPositionAction.h>
#include <iiwa_msgs/JointPosition.h> 

namespace joint_follower {

class JointFollower : public iimoveit::RobotInterface {
public:
  JointFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_factor, double max_radius, bool rad_input, std::string name)
      : RobotInterface(node_handle, planning_group, base_frame),
        scale_factor_(scale_factor),
        max_radius_(max_radius),
        max_radius2_(max_radius*max_radius),
    		as_(nh_, name, boost::bind(&JointFollower::executeCB, this, _1), false),
    		action_name_(name),
				rad_input_(rad_input),
				first_time_(true) {

		if(rad_input_) {
			angle_conversion_ = 1.0;	
		}
		else {
			angle_conversion_ = 3.1416/180.0;
		}

		// use when initial joint positions are given
		initial_joint_positions_.joint_names.resize(7);
		initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		initial_joint_positions_.points.resize(1);
		initial_joint_positions_.points[0].positions.resize(7);
//		initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -50.21;
//		initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * 31.60;
//		initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * 44.26;
//		initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -83.30;
//		initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * -19.60; 
//		initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * -5.25; 
//		initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 1.01;

    // new iiwa mounting and mirroring the operator's input
		initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -1.0 * -28.03;
		initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-1.0 * 19.18 + 90.0);
		initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -22.67;
		initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -1.0 * -60.41;
		initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (58.19 + 90.0); 
		initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * -1.0 * -12.01; 
		initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * -6.46;

		mcs_initial_joint_positions_.joint_names.resize(7);
		mcs_initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		mcs_initial_joint_positions_.points.resize(1);
		mcs_initial_joint_positions_.points[0].positions.resize(7);

		upper_joint_limits_.resize(7);
		lower_joint_limits_.resize(7);
		upper_joint_limits_[0] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[0] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[1] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[1] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[2] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[2] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[3] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[3] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[4] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[4] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[5] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[5] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[6] = 3.1416/180.0 * 175.0;
		lower_joint_limits_[6] = 3.1416/180.0 * -175.0;
		
		as_.start();
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }

  void registerSubscriberRelative(const std::string& topic) {
    joint_subscriber_ = node_handle_->subscribe(topic, 1, &JointFollower::jointCallbackRelative, this);
  }

  void registerSubscriberAbsolute(const std::string& topic) {
    joint_subscriber_ = node_handle_->subscribe(topic, 1, &JointFollower::jointCallbackAbsolute, this);
  }

  void setBasePose(const geometry_msgs::Pose& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::Pose getBasePose() {
    return base_pose_;
  }

	void setBasePoseJointPositions(const std::vector<std::string>& names, const std::vector<double>& initial_angles) {
		// header missing?
		
		base_pose_joint_positions_.joint_names.resize(7);
		base_pose_joint_positions_.joint_names = names;
//		base_pose_joint_positions_.joint_names[0] = "iiwa_joint_1";
//		ROS_INFO_NAMED("joint_follower", "nach names1");
//		base_pose_joint_positions_.joint_names[2] = "iiwa_joint_3";
//		base_pose_joint_positions_.joint_names[3] = "iiwa_joint_4";
//		base_pose_joint_positions_.joint_names[4] = "iiwa_joint_5";
//		base_pose_joint_positions_.joint_names[5] = "iiwa_joint_6";
//		base_pose_joint_positions_.joint_names[6] = "iiwa_joint_7";
		base_pose_joint_positions_.points.resize(1);
		base_pose_joint_positions_.points[0].positions.resize(7);
		base_pose_joint_positions_.points[0].positions = initial_angles;
//		base_pose_joint_positions_.points[0].positions[0] = initial_angles[0];
//		base_pose_joint_positions_.points[0].positions[1] = initial_angles[1];
//		base_pose_joint_positions_.points[0].positions[2] = initial_angles[2];
//		base_pose_joint_positions_.points[0].positions[3] = initial_angles[3];
//		base_pose_joint_positions_.points[0].positions[4] = initial_angles[4];
//		base_pose_joint_positions_.points[0].positions[5] = initial_angles[5];
//		base_pose_joint_positions_.points[0].positions[6] = initial_angles[6];

		base_pose_joint_positions_.points[0].time_from_start = ros::Duration(0.01); // later variable
	}

	void moveToInitialJointPositions() {
		planAndMove(initial_joint_positions_.points[0].positions, std::string("initial joint positions"));
	}

	void setBasePoseToCurrent() {
		base_pose_ = getPose(std::string("iiwa_link_ee")).pose;	
	}

  void executeCB(const test_trajectories_automation::MoveToJointPositionGoalConstPtr &goal)
  {
    // helper variables
//    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
//    ROS_INFO("%s: Executing callback. Printing sent joint positions.", action_name_.c_str());

//		ROS_INFO("%f", goal->joint_positions.points[0].positions[0]);

//		result_.test = 13.37;
//		as_.setSucceeded(result_);

		moveToGoalJointPositions(goal->joint_positions);
		setBasePoseToCurrent();
    setBasePoseJointPositions(getJointNames(), getJointPositions());
    first_time_ = true;
	}

	void moveToGoalJointPositions(const trajectory_msgs::JointTrajectory goal_joint_positions ) {
		planAndMove(goal_joint_positions.points[0].positions, std::string("goal joint positions"), false);
	}


private:
  ros::Subscriber joint_subscriber_;
	trajectory_msgs::JointTrajectory initial_joint_positions_;
  geometry_msgs::Pose base_pose_;
	trajectory_msgs::JointTrajectory base_pose_joint_positions_; 
	trajectory_msgs::JointTrajectory mcs_initial_joint_positions_;
  double scale_factor_;
  double max_radius_;
  double max_radius2_;
	bool rad_input_;
	double angle_conversion_;
	bool first_time_;
	std::vector<double> upper_joint_limits_;
	std::vector<double> lower_joint_limits_;

  void jointCallbackRelative(const iiwa_msgs::JointPosition::ConstPtr& msg) {
			double a1 = msg->position.a1; // format -> jointAngles receive/read script
			double a2 = msg->position.a2;
			double a3 = msg->position.a3;
			double a4 = msg->position.a4;
			double a5 = msg->position.a5;
			double a6 = msg->position.a6;
			double a7 = msg->position.a7;

			if(first_time_) {
				mcs_initial_joint_positions_.points[0].positions[0] = a1;
				mcs_initial_joint_positions_.points[0].positions[1] = a2;
				mcs_initial_joint_positions_.points[0].positions[2] = a3;
				mcs_initial_joint_positions_.points[0].positions[3] = a4;
				mcs_initial_joint_positions_.points[0].positions[4] = a5; 
				mcs_initial_joint_positions_.points[0].positions[5] = a6; 
				mcs_initial_joint_positions_.points[0].positions[6] = a7;				
				first_time_ = false;
			}

      //geometry_msgs::Pose target_pose = base_pose_;
			trajectory_msgs::JointTrajectory trajectory_point = base_pose_joint_positions_;
//			trajectory_point.points[0].positions[0] += (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
//			trajectory_point.points[0].positions[1] += (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
//			trajectory_point.points[0].positions[2] -= (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
//			trajectory_point.points[0].positions[3] += (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
//			trajectory_point.points[0].positions[4] -= (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
//			trajectory_point.points[0].positions[5] += (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;
//			trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

      // new mounting and mirroring movement
			trajectory_point.points[0].positions[0] -= (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
			trajectory_point.points[0].positions[1] -= (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
			trajectory_point.points[0].positions[2] += (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
			trajectory_point.points[0].positions[3] -= (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
			trajectory_point.points[0].positions[4] += (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
			trajectory_point.points[0].positions[5] -= (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;
			trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

			trajectory_point = jointLimitation(trajectory_point);

      publishTrajectory(trajectory_point);
//    }
  }

  void jointCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    publishPoseGoal(msg->pose, 0.01);
  }

	trajectory_msgs::JointTrajectory jointLimitation(trajectory_msgs::JointTrajectory set_value) {
		for (int i=0; i<7; i++) {
			if(set_value.points[0].positions[i] > upper_joint_limits_[i]) {
				set_value.points[0].positions[i] = upper_joint_limits_[i];
			}
			else if	(set_value.points[0].positions[i] < lower_joint_limits_[i]) {
				set_value.points[0].positions[i] = lower_joint_limits_[i];
			}		
		}
		return set_value;   
	}

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<test_trajectories_automation::MoveToJointPositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  test_trajectories_automation::MoveToJointPositionFeedback feedback_;
  test_trajectories_automation::MoveToJointPositionResult result_;

};
} // namespace pose_follower


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectories_automation_joint_follower");
	ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();	

	double scale_factor; // MOD
	node_handle.param("/iiwa/joint_follower/scale_factor", scale_factor, 1.0);

//  action_server::MoveToJointPositionAction as_pose_follower("as_pose_follower");
//  action_server::MoveToJointPositionAction as_pose_follower("as_pose_follower", &node_handle, "manipulator", "world");
  joint_follower::JointFollower joint_follower(&node_handle, "manipulator", "world", scale_factor, 2, true, "as_pose_follower"); // as_pose_follower -> to work with client from pose follower

//	pose_follower.moveToInitialJointPositions();
//	pose_follower.setBasePoseToCurrent();

//  pose_follower.waitForApproval();
  joint_follower.registerSubscriberRelative(std::string("/iiwa/jointAnglesFromFile/JointPositionRelative"));
  ROS_INFO_NAMED("pose_follower", "Subscribed to joint angles from file!");

  ros::Rate rate(100);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "test_trajectories_automation_follower");

//  ros::AsyncSpinner spinner(1);
//  spinner.start();	

//  MoveToJointPositionAction pose_follower("pose_follower");

//  ros::Rate rate(10);
//  while(ros::ok()) {
//    rate.sleep();
//  }
//  ros::shutdown();
//  return 0;
//}

