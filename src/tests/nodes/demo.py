#!/usr/bin/env python
import rospy
import rostest
import unittest
import sys
import time
from sensor_msgs.msg import LaserScan

__author__ = ''

PKG = 'tests'
NAME = 'lap_time_test'

class TesterClass(unittest.TestCase):
    
    def __init__(self, *args):
        self.message_received = False
        self.message_received_raw = None
        rospy.init_node(NAME, anonymous=True)
        super(TesterClass, self).__init__(*args)

    def test_passed(self): # test names must start with 'test_'
        sub = rospy.Subscriber('/base_scan', LaserScan, self.callback)

        while not self.message_received:
            time.sleep(0.1)

        self.assertTrue(69.69 > self.message_received_raw)

    def callback(self, msg):
        self.message_received = True
        self.message_received_raw = 69.0

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TesterClass, sys.argv)
