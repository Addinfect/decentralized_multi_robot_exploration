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
        self.timestamp = 0
        self.timestamp_set = False
        
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/exploration_percentage', Float32, self.callback)
      
        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue(True)

    def callback(self, msg):
        self.message_received = msg.data >= 99.5

        #set Goal Reached after 30 sec after hitting 95.0 %
        if (not self.message_received and msg.data >= 95.0):
            if not self.timestamp_set:
                self.timestamp = rospy.Time.now().to_sec()
                self.timestamp_set = True
            if self.timestamp+60 <= rospy.Time.now().to_sec():
                self.message_received = True

   
if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
