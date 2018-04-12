#!/usr/bin/env python
import sys
import csv
import rospy
from iiwa_msgs.msg import JointPosition
import actionlib
import test_trajectories_automation.msg

SAMPLE_RATE = 100

class PublishJointAnglesAction(object):
    # create messages that are used to publish feedback/result
    _feedback = test_trajectories_automation.msg.PublishTrajectoryFeedback()
    _result = test_trajectories_automation.msg.PublishTrajectoryResult()

    def __init__(self, name, topicName='JointAngles'):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, test_trajectories_automation.msg.PublishTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)

        self.pub = rospy.Publisher('jointAnglesFromFile/'+topicName, JointPosition, queue_size=10)
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
        reader = csv.reader(open(filename))
        for line in reader:
            if rospy.is_shutdown(): break
            values = [float(x) for x in line]
            pose = JointPosition()
            pose.header.frame_id = "/world"
            pose.header.stamp = rospy.Time.now()
            pose.position.a1 = values[0]
            pose.position.a2 = values[1]
            pose.position.a3 = values[2]
            pose.position.a4 = values[3]
            pose.position.a5 = values[4]
            pose.position.a6 = values[5]
            pose.position.a7 = values[6]
            self.pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('as_publish_trajectory')
    server = PublishJointAnglesAction(rospy.get_name(), "JointPositionRelative")
    rospy.spin()
