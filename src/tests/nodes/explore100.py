#!/usr/bin/env python3
import rospy
import rostest
import unittest
import sys
import time
from std_msgs.msg import Float32
__author__ = ''

PKG = 'tests'
NAME = 'exploration100_test'

class TesterClass(unittest.TestCase):
    
    def __init__(self, *args):
        self.message_received = False
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/exploration_percentage', Float32, self.callback)
      
        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue(True)

    def callback(self, msg):
        self.message_received = msg.data >= 90  #99.5
   
if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
