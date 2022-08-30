#!/usr/bin/env python
from pkgutil import ImpImporter
from socket import MsgFlag
import rospy
import rostest
import unittest
import sys
import time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
from std_msgs.msg import Header
__author__ = ''

PKG = 'tests'
NAME = 'simple_navigation_test'

class TesterClass(unittest.TestCase):
    
    def __init__(self, *args):
        self.message_received = False
        self.message_received_raw = None
        self.goal_message = None
        self.goal_published = False
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.callback)
        sub_t = rospy.Subscriber('/clock', Clock, self.time_callback)
        self.goal_message = PoseStamped()
        self.goal_message.header = Header()
        self.goal_message.pose.position.x = 8.2
        self.goal_message.pose.position.y = 2.7
        self.goal_message.pose.position.z = 0.0        
        self.goal_message.pose.orientation.x = 0.00
        self.goal_message.pose.orientation.y = 0.00
        self.goal_message.pose.orientation.z = 0.00
        self.goal_message.pose.orientation.w = 1.0
        self.goal_message.header.frame_id = "map"
        self.goal_message.header.seq = 1
        self.goal_message.header.stamp = rospy.Time.now()
        

        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue("Goal reached." ==  self.message_received_raw,msg=self.message_received_raw)

    def callback(self, msg):
        if msg.status.status != 2 and msg.status.status != 8:  # 2 --> Goal was overwrite | ignore error
            self.message_received_raw = msg.status.text
            self.message_received = True
    def time_callback(self, msg):
        if msg.clock.secs > 120.0 and not self.goal_published:
            self.push_goal()
            self.goal_published = True
        if msg.clock.secs%180 == 0.0 and self.goal_published:
            self.goal_published = False
    def push_goal(self):
        pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
        #pub.publish(self.goal_message)

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
