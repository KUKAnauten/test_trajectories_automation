#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <sstream>
#include <iimoveit/robot_interface.h>
#include <iiwa_msgs/JointPosition.h> 
#include <iiwa_msgs/JointVelocity.h> 

std_msgs::UInt32 msg_status_word;
std_msgs::Float64 msg_time;
//std_msgs::Float64 msg_last_time;
//std_msgs::Float64 msg_time_delta;
//double joint_velocity[7];
trajectory_msgs::JointTrajectory msg_joint_command;
//std::vector<double> position_command;
//iiwa_msgs::JointVelocity msg_calculated_velocity;
//iiwa_msgs::JointPosition msg_joint_position;

sensor_msgs::JointState msg_joint_states;
geometry_msgs::PoseStamped msg_pose_command;


void statusCallback (const std_msgs::UInt32::ConstPtr& msg) {
  msg_status_word = *msg;  
}

void jointStatesCallback (const sensor_msgs::JointState::ConstPtr& msg) {
  msg_joint_states = *msg;
}

//void jointStatesCallback (const sensor_msgs::JointState::ConstPtr& msg) {
//  for (int i=0; i<7; i++) {  
//    joint_velocity[i] = msg->velocity[i];
//  }  
//}

void jointCommandCallback (const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  msg_joint_command = *msg;
//  for (int i=0; i<7; i++) {
//  position_command = msg->points[0].positions;
//  } 
//  std::vector<double> joint_positions;
//  joint_positions = ri_obj.getJointPositions();
}

void poseCommandCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
  msg_pose_command = *msg;  
}

//std::vector<double> calculateVelocity(const std::vector<double> position, const std::vector<double> last_position, const double time, const double last_time) {
//  std::vector<double> velocity;
//  velocity.resize(7);
//  for (int i=0; i<7; i++) {
//    velocity[i] = (position[i]-last_position[i])/(time-last_time);
//  }
//  return velocity;
//}

//std::vector<double> calculateCommandedVelocity(const std::vector<double> pos_cmd, const std::vector<double> current_pos, const double delta_t) {
//  std::vector<double> vel_cmd;
//  vel_cmd.resize(7);
//  for (int i=0; i<7; i++) {
//    vel_cmd[i] = (pos_cmd[i]-current_pos[i])/delta_t;
//  }  
//  return vel_cmd;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Subscriber	
  ros::Subscriber status_sub = node_handle.subscribe("/iiwa/status_word", 1, statusCallback);
  ros::Subscriber joint_states_sub = node_handle.subscribe("/iiwa/joint_states", 1, jointStatesCallback);
  ros::Subscriber joint_command_sub = node_handle.subscribe("/iiwa/PositionJointInterface_trajectory_controller/command", 1, jointCommandCallback);
  ros::Subscriber pose_command_sub = node_handle.subscribe("/iiwa/poseFromFile/PoseStampedRelative", 1, poseCommandCallback);

  // Publisher
  ros::Publisher status_pub = node_handle.advertise<std_msgs::UInt32>("published_status", 10);
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("published_pose", 10);
  ros::Publisher pose_command_pub = node_handle.advertise<geometry_msgs::PoseStamped>("published_pose_command", 10);  
//  ros::Publisher joint_pub = node_handle.advertise<iiwa_msgs::JointPosition>("published_joints", 10);
//  ros::Publisher joint_velocity_pub = node_handle.advertise<iiwa_msgs::JointVelocity>("published_joint_velocity", 10);
  ros::Publisher joint_states_pub = node_handle.advertise<sensor_msgs::JointState>("published_joint_states", 10);
  ros::Publisher joint_command_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("published_joint_command", 10);

//  ros::Publisher counter_pub = node_handle.advertise<std_msgs::UInt32>("counter", 10);

	iimoveit::RobotInterface ri_object(&node_handle, "manipulator", "world");

  // Debug
//  ros::Publisher time_pub = node_handle.advertise<std_msgs::Float64>("published_time", 10);
//  ros::Publisher last_time_pub = node_handle.advertise<std_msgs::Float64>("published_last_time", 10);
//  ros::Publisher time_delta_pub = node_handle.advertise<std_msgs::Float64>("published_time_delta", 10);
//  ros::Subscriber command_sub = node_handle.subscribe("/iiwa/PositionJointInterface_trajectory_controller/command", 1, commandCallback);
//  ros::Subscriber command_sub = node_handle.subscribe("/iiwa/PositionJointInterface_trajectory_controller/command", 1, boost::bind(commandCallback, _1, ri_object));
//  ros::Publisher command_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("published_command", 10);
//  ros::Publisher vel_command_pub = node_handle.advertise<iiwa_msgs::JointVelocity>("published_velocity_command", 10);


  ros::Rate loop_rate(50);

  geometry_msgs::PoseStamped msg_pose;
//  iiwa_msgs::JointPosition msg_joints;
//  std::vector<double> joint_positions;
//  std::vector<double> last_joint_positions {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//  std::vector<double> joint_velocity;  
//  iiwa_msgs::JointVelocity msg_joint_velocity; 
//  std::vector<double> commanded_velocity;
//  iiwa_msgs::JointVelocity msg_commanded_velocity;
//  position_command.resize(7);
  
  double time;
//  double last_time = 0.0;

  while (ros::ok())
  {
//    time = ros::Time::now().toSec();
    msg_pose = ri_object.getPose(std::string("iiwa_link_ee"));
//    joint_positions = ri_object.getJointPositions();
//    msg_joints.position.a1 = joint_positions[0];
//    msg_joints.position.a2 = joint_positions[1];
//    msg_joints.position.a3 = joint_positions[2];
//    msg_joints.position.a4 = joint_positions[3];
//    msg_joints.position.a5 = joint_positions[4];
//    msg_joints.position.a6 = joint_positions[5];
//    msg_joints.position.a7 = joint_positions[6];
//    
//    joint_velocity = calculateVelocity(joint_positions, last_joint_positions, time, last_time);
//    
//    msg_joint_velocity.velocity.a1 = joint_velocity[0];
//    msg_joint_velocity.velocity.a2 = joint_velocity[1];
//    msg_joint_velocity.velocity.a3 = joint_velocity[2];
//    msg_joint_velocity.velocity.a4 = joint_velocity[3];
//    msg_joint_velocity.velocity.a5 = joint_velocity[4];
//    msg_joint_velocity.velocity.a6 = joint_velocity[5];
//    msg_joint_velocity.velocity.a7 = joint_velocity[6];
    
//    commanded_velocity = calculateCommandedVelocity(position_command, joint_positions, 0.01);

//    msg_commanded_velocity.velocity.a1 = commanded_velocity[0];
//    msg_commanded_velocity.velocity.a2 = commanded_velocity[1];
//    msg_commanded_velocity.velocity.a3 = commanded_velocity[2];
//    msg_commanded_velocity.velocity.a4 = commanded_velocity[3];
//    msg_commanded_velocity.velocity.a5 = commanded_velocity[4];
//    msg_commanded_velocity.velocity.a6 = commanded_velocity[5];
//    msg_commanded_velocity.velocity.a7 = commanded_velocity[6];

//    msg_time.data = time;
//    msg_last_time.data = last_time;
//    msg_time_delta.data = time - last_time;

    status_pub.publish(msg_status_word);
    pose_pub.publish(msg_pose);
    pose_command_pub.publish(msg_pose_command);
    joint_states_pub.publish(msg_joint_states);
    joint_command_pub.publish(msg_joint_command);
//    joint_pub.publish(msg_joints);
//    joint_velocity_pub.publish(msg_joint_velocity);
//    command_pub.publish(msg_command);
//    vel_command_pub.publish(msg_commanded_velocity);

//    time_pub.publish(msg_time);
//    last_time_pub.publish(msg_last_time);
//    time_delta_pub.publish(msg_time_delta);

//    last_joint_positions = joint_positions;
//    last_time = time;


    
//    vel_command_pub.publish(msg_calculated_velocity);




    loop_rate.sleep();
  }
	ros::shutdown();
  return 0;
}
