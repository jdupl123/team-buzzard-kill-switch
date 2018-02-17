#!/usr/bin/env python
# license removed for brevity
import rospy
from nmea_msgs.msg import Sentence
from state_estimator import StateEstimator
from nav_msgs.msg import Odometry

state_estimator = StateEstimator()

def callback_passthrough( sentence_msg ):
    state_estimator.state_estimator(sentence_msg.sentence)

pub = rospy.Publisher('state', Odometry, queue_size=10)
sub = rospy.Subscriber('nmea_sentence', Sentence, callback_passthrough)

def gps_node():
   rospy.init_node('gps_node', anonymous=True)
   rospy.spin()
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
       msg = Odometry()
       msg.pose.pose.position.x = state_estimator.state['x']
       msg.pose.pose.position.y = state_estimator.state['y']
       msg.pose.pose.position.z = state_estimator.state['bearing']
       msg.twist.twist.linear.z = state_estimator.state['velocity']
       msg.pose.covariance[0] = state_estimator.state['fix']
       pub.publish(msg)
       rate.sleep()

if __name__ == '__main__':
   try:
       gps_node()
   except rospy.ROSInterruptException:
       pass