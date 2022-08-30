#!/usr/bin/env python
import rospy
import rostest
import unittest
import sys
import time
from rosgraph_msgs.msg import Clock

__author__ = ''

PKG = 'tests'
NAME = 'timer_test'

class TesterClass(unittest.TestCase):
    
    def __init__(self, *args):
        self.time = 0.0
        for x in args:
            if type(x) == int or type(x) == float:
                self.time = x
            else:
                self.time = 600.0
                break
        
        self.message_received = False
        self.message_received_raw = None
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/clock', Clock, self.callback)

        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue(self.time < self.message_received_raw)

    def callback(self, msg):
        self.message_received_raw = msg.clock.secs
        self.message_received = self.time < self.message_received_raw

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
