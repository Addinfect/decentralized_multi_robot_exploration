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
from std_msgs.msg import Header, String
__author__ = ''

PKG = 'tests'
NAME = 'exploration_test'

class TesterClass(unittest.TestCase):
    
    def __init__(self, *args):
        self.message_received = False
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/frontier/finished', String, self.callback)
      
        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue(True)

    def callback(self, msg):
        self.message_received = True
   
if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
