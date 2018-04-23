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

#define PI 3.14159265359

namespace pose_follower {

class PoseFollower : public iimoveit::RobotInterface {
public:
  PoseFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_x, double scale_y, double scale_z, double scale_rot_x, double scale_rot_y, double scale_rot_z, double max_radius, std::string name)
      : RobotInterface(node_handle, planning_group, base_frame),
        scale_x_(scale_x),
        scale_y_(scale_y),
        scale_z_(scale_z),
        scale_rot_x_(scale_rot_x),
        scale_rot_y_(scale_rot_y),
        scale_rot_z_(scale_rot_z),
        max_radius_(max_radius),
        max_radius2_(max_radius*max_radius),
    		as_(nh_, name, boost::bind(&PoseFollower::executeCB, this, _1), false),
    		action_name_(name),
        first_time_(true) {


		// use when initial joint positions are given
		initial_joint_positions_.joint_names.resize(7);
		initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		initial_joint_positions_.points.resize(1);
		initial_joint_positions_.points[0].positions.resize(7);
		initial_joint_positions_.points[0].positions[0] = PI/180.0 * -50.21;
		initial_joint_positions_.points[0].positions[1] = PI/180.0 * 31.60;
		initial_joint_positions_.points[0].positions[2] = PI/180.0 * 44.26;
		initial_joint_positions_.points[0].positions[3] = PI/180.0 * -83.30;
		initial_joint_positions_.points[0].positions[4] = PI/180.0 * -19.60; 
		initial_joint_positions_.points[0].positions[5] = PI/180.0 * -5.25; 
		initial_joint_positions_.points[0].positions[6] = PI/180.0 * 1.01;
		
		as_.start();
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }

  void registerSubscriberRelative(const std::string& topic) {
    pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackRelative, this);
  }

  void registerSubscriberAbsolute(const std::string& topic) {
    pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackAbsolute, this);
  }

  void setBasePose(const geometry_msgs::Pose& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::Pose getBasePose() {
    return base_pose_;
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
    first_time_ = true;
	}

	void moveToGoalJointPositions(const trajectory_msgs::JointTrajectory goal_joint_positions ) {
		planAndMove(goal_joint_positions.points[0].positions, std::string("goal joint positions"), false);
	}


private:
  ros::Subscriber pose_subscriber_;
	trajectory_msgs::JointTrajectory initial_joint_positions_;
  geometry_msgs::Pose base_pose_;
  double scale_x_;
  double scale_y_;
  double scale_z_;
  double scale_rot_x_;
  double scale_rot_y_;
  double scale_rot_z_;
  double max_radius_;
  double max_radius2_;
  bool first_time_;
  tf::Quaternion calib_quaternion_; // relation between initial poses of iiwa and mcs 

  void poseCallbackRelative(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double x = msg->pose.position.x * scale_x_;
    double y = msg->pose.position.y * scale_y_;
    double z = msg->pose.position.z * scale_z_;
    if (x*x + y*y + z*z <= max_radius2_) {
      
      tf::Quaternion base_quaternion(base_pose_.orientation.x, base_pose_.orientation.y, base_pose_.orientation.z, base_pose_.orientation.w);
      tf::Quaternion next_quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      
      if (first_time_){
        calib_quaternion_ = inverse(next_quaternion);    
        first_time_ = false;
      }  

      tf::Quaternion relative_quaternion = next_quaternion * calib_quaternion_;

      if ( (scale_rot_x_ != 1.0) || (scale_rot_y_ != 1.0) || (scale_rot_z_ != 1.0) ) {
        tf::Matrix3x3 rotMatrix(relative_quaternion);
        double euler_x, euler_y, euler_z;
        rotMatrix.getEulerYPR(euler_z, euler_y, euler_x);
        euler_x *= scale_rot_x_;
        euler_y *= scale_rot_y_;
        euler_z *= scale_rot_z_;
        rotMatrix.setEulerYPR(euler_z, euler_y, euler_x);
        rotMatrix.getRotation(relative_quaternion);

//        double roll, pitch, yaw;
//        rotMatrix.getRPY(roll, pitch, yaw);
//        roll *= scale_rot_x_;
//        pitch *= scale_rot_y_;
//        yaw *= scale_rot_z_;
//        relative_quaternion.setRPY(roll, pitch, yaw);
      }

      relative_quaternion = relative_quaternion * base_quaternion;

      geometry_msgs::Pose target_pose = base_pose_;
      target_pose.position.x += x;
      target_pose.position.y += y;
      target_pose.position.z += z;
      target_pose.orientation.x = relative_quaternion.getX();
      target_pose.orientation.y = relative_quaternion.getY();
      target_pose.orientation.z = relative_quaternion.getZ();
      target_pose.orientation.w = relative_quaternion.getW();
      publishPoseGoal(target_pose, 0.01);

//        tf::Quaternion base_quaternion(base_pose_.orientation.x, base_pose_.orientation.y, base_pose_.orientation.z, base_pose_.orientation.w);
//        tf::Quaternion next_quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
//        tf::Quaternion result_quaternion = next_quaternion * base_quaternion;

//        geometry_msgs::Pose target_pose = base_pose_;
//        target_pose.position.x += x;
//        target_pose.position.y += y;
//        target_pose.position.z += z;
//        target_pose.orientation.x = result_quaternion.getX();
//        target_pose.orientation.y = result_quaternion.getY();
//        target_pose.orientation.z = result_quaternion.getZ();
//        target_pose.orientation.w = result_quaternion.getW();
//        publishPoseGoal(target_pose, 0.01);
    }
  }

  void poseCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    publishPoseGoal(msg->pose, 0.01);
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
  ros::init(argc, argv, "test_trajectories_automation_pose_follower");
	ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();	

	double scale_x, scale_y, scale_z, scale_rot_x, scale_rot_y, scale_rot_z; 
	node_handle.param("/iiwa/pose_follower_server/scale_x", scale_x, 1.0);
  node_handle.param("/iiwa/pose_follower_server/scale_y", scale_y, 1.0);
	node_handle.param("/iiwa/pose_follower_server/scale_z", scale_z, 1.0);
  node_handle.param("/iiwa/pose_follower_server/scale_rot_x", scale_rot_x, 1.0);
	node_handle.param("/iiwa/pose_follower_server/scale_rot_y", scale_rot_y, 1.0);
  node_handle.param("/iiwa/pose_follower_server/scale_rot_z", scale_rot_z, 1.0);

//  action_server::MoveToJointPositionAction as_pose_follower("as_pose_follower");
//  action_server::MoveToJointPositionAction as_pose_follower("as_pose_follower", &node_handle, "manipulator", "world");
  pose_follower::PoseFollower pose_follower(&node_handle, "manipulator", "world", scale_x, scale_y, scale_z, scale_rot_x, scale_rot_y, scale_rot_z, 2, "as_pose_follower");

//	pose_follower.moveToInitialJointPositions();
//	pose_follower.setBasePoseToCurrent();

//  pose_follower.waitForApproval();
  pose_follower.registerSubscriberRelative(std::string("/iiwa/poseFromFile/PoseStampedRelative"));
  ROS_INFO_NAMED("pose_follower", "Subscribed to pose!");

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

