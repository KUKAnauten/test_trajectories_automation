#!/usr/bin/env python
import sys
import csv
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
import test_trajectories_automation.msg

SAMPLE_RATE = 100

class PublishPoseAction(object):
    # create messages that are used to publish feedback/result
    _feedback = test_trajectories_automation.msg.PublishTrajectoryFeedback()
    _result = test_trajectories_automation.msg.PublishTrajectoryResult()

    def __init__(self, name, topicName='PoseStamped'):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, test_trajectories_automation.msg.PublishTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)

        self.pub = rospy.Publisher('poseFromFile/'+topicName, PoseStamped, queue_size=10)
#				rospy.init_node('posePublisher', anonymous=True)
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
        
#        # append the seeds for the fibonacci sequence
#        self._feedback.sequence = []
#        self._feedback.sequence.append(0)
#        self._feedback.sequence.append(1)
#        
#        # publish info to the console for the user
#        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
#        
#        # start executing the action
#        for i in range(1, goal.order):
#            # check that preempt has not been requested by the client
#            if self._as.is_preempt_requested():
#                rospy.loginfo('%s: Preempted' % self._action_name)
#                self._as.set_preempted()
#                success = False
#                break
#            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
#            # publish the feedback
#            self._as.publish_feedback(self._feedback)
#            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
#            r.sleep()
#          
#        if success:
#            self._result.sequence = self._feedback.sequence
#            rospy.loginfo('%s: Succeeded' % self._action_name)
#            self._as.set_succeeded(self._result)

    def talker(self, filename):
#				pub = rospy.Publisher('poseFromFile/'+topicName, PoseStamped, queue_size=10)
#				rospy.init_node('posePublisher', anonymous=True)
#				rate = rospy.Rate(SAMPLE_RATE)
        reader = csv.reader(open(filename))
        for line in reader:
				    if rospy.is_shutdown(): break
				    values = [float(x) for x in line]
				    #rospy.loginfo(values)
				    pose = PoseStamped()
				    pose.header.frame_id = "/world"
				    pose.header.stamp = rospy.Time.now()
				    pose.pose.position.x = values[0]
				    pose.pose.position.y = values[1]
				    pose.pose.position.z = values[2]
				    pose.pose.orientation.x = values[3]
				    pose.pose.orientation.y = values[4]
				    pose.pose.orientation.z = values[5]
				    pose.pose.orientation.w = values[6]
				    self.pub.publish(pose)
				    self.rate.sleep()

if __name__ == '__main__':
#    if len(sys.argv) >  4 or len(sys.argv) < 2:
#        print 'Usage: ' + 'filename [sample rate] [topicName]'
#    elif len(sys.argv) == 2:
#        try:
#            talker(sys.argv[1])
#        except rospy.ROSInterruptException:
#            pass
#    elif len(sys.argv) == 3:
#        try:
#            SAMPLE_RATE = float(sys.argv[2])
#            talker(sys.argv[1])
#        except rospy.ROSInterruptException:
#            pass
#    elif len(sys.argv) == 4:
#        try:
#            SAMPLE_RATE = float(sys.argv[2])
#            talker(sys.argv[1], sys.argv[3])
#        except rospy.ROSInterruptException:
#            pass
    rospy.init_node('as_publish_trajectory')
    server = PublishPoseAction(rospy.get_name(), "PoseStampedRelative")
    rospy.spin()
