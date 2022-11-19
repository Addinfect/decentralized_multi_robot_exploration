#!/usr/bin/env python3

import rospy, sys, math, csv, os
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from frontier_exploration.msg import TimeLog


class Time_Tracker():
    
    def __init__(self, argv, argc):
        rospy.init_node("Time_Tracker", anonymous=True)

        print(argv)
        self.worldname = argv[1]
        self.num_Robots = int(argv[2])
        self.algorithmus = argv[3]
        self.exploring_size = 0
        self.explored_cells = 0


        rospy.Subscriber('/timing', TimeLog, self.callback_time) 
        self.start_time = 0.0
        self.run_finished = False
        self.start_time_initialized = False


    def callback_time(self, msg):
        self.save_data_from_run(msg.time,msg.state)


    def save_data_from_run(self, time, stage):
        with open('timing_results.csv', 'a+') as csvfile:
            writer = csv.writer(csvfile, dialect='excel', delimiter=' ',
                                    quotechar=';', quoting=csv.QUOTE_MINIMAL)
            """
            writer.writerow(['World'] + ['Number_Robots'] + ['Assigner'] + ['Stage'] + ['Time'])
            """
            rospy.loginfo_once("TestTrackerPath: "+str(os.path.dirname(__file__)))
            row = [self.worldname, str(self.num_Robots), self.algorithmus, stage, time]
            writer.writerow(row)


def main(argv, argc):
    explor_tracker = Time_Tracker(argv, argc)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))
