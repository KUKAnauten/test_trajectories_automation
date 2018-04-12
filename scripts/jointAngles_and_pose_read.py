#!/usr/bin/env python
import sys
import csv
import rospy
from iiwa_msgs.msg import JointPosition
from geometry_msgs.msg import PoseStamped
import actionlib
import test_trajectories_automation.msg

SAMPLE_RATE = 100

class PublishJointAnglesAction(object):
    # create messages that are used to publish feedback/result
    _feedback = test_trajectories_automation.msg.PublishTrajectoryFeedback()
    _result = test_trajectories_automation.msg.PublishTrajectoryResult()

    def __init__(self, name, jointTopicName='JointAngles', poseTopicName='PoseStamped'):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, test_trajectories_automation.msg.PublishTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)

        self.joint_pub = rospy.Publisher('jointAnglesFromFile/'+jointTopicName, JointPosition, queue_size=10)
        self.pose_pub = rospy.Publisher('poseFromFile/'+poseTopicName, PoseStamped, queue_size=10)
        self.rate = rospy.Rate(SAMPLE_RATE)

        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        success = True
				
        filename = goal.filename
        self.talker(filename)

        if success:
            self._result.test = 13.37
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def talker(self, filename):
        joint_reader = csv.reader(open(filename))
#        pose_reader = csv.reader(open(filename_pose))
        for line in joint_reader:
            if rospy.is_shutdown(): break
            values = [float(x) for x in line]
            joints = JointPosition()
            joints.header.frame_id = "/world"
            joints.header.stamp = rospy.Time.now()
            joints.position.a1 = values[0]
            joints.position.a2 = values[1]
            joints.position.a3 = values[2]
            joints.position.a4 = values[3]
            joints.position.a5 = values[4]
            joints.position.a6 = values[5]
            joints.position.a7 = values[6]
            
            pose = PoseStamped()
            pose.header.frame_id = "/world"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = values[7]
            pose.pose.position.y = values[8]
            pose.pose.position.z = values[9]
            pose.pose.orientation.x = values[10]
            pose.pose.orientation.y = values[11]
            pose.pose.orientation.z = values[12]
            pose.pose.orientation.w = values[13]                

            self.joint_pub.publish(joints)
            self.pose_pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('as_publish_trajectory')
    server = PublishJointAnglesAction(rospy.get_name(), "JointPositionRelative", "PoseStampedRelative")
    rospy.spin()
