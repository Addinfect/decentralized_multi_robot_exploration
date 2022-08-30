#!/usr/bin/env python3


import rospy, math, csv, os, datetime
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Float32MultiArray

rospy.init_node('ResultTestTracker', anonymous=True)


class InformationTracker:
    def __init__(self) -> None:
        rospy.Subscriber('/traveled_distance', Float32MultiArray, self.callback_distance)
        rospy.Subscriber('/exploration_percentage', Float32, self.callback_map)

        self.nextInfoThreshold = 0.0
        self.distances = 0.0
        self.percentage = 0.0

    def callback_distance(self, msg:Float32MultiArray):
        self.distances = msg.data
    
    def callback_map(self, msg:Float32):
        self.percentage = msg.data

    def get_traveled_distance(self) -> float:
        return self.distances

    def get_explored_part(self) -> int:
        return self.percentage

tracker = InformationTracker()

rate = rospy.Rate(5)
date_time = (datetime.datetime.now().strftime('%d-%m-%Y-%H-%M-%S'))
with open('result_data%s.csv' %date_time, 'w+') as csvfile:
    writer = csv.writer(csvfile, dialect='excel', delimiter=' ',
                            quotechar=';', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['Time'] + ['Explored_Map_percentage'] + ['Traveled_Distance'] + ['Robots_distances'])

    rospy.loginfo_once("TestTrackerPath: "+str(os.path.dirname(__file__)))
    start_time = rospy.Time.now().to_sec()
    
    infothreshold = 0.01

    while not rospy.is_shutdown():
        percentage = tracker.get_explored_part()
        distances = tracker.get_traveled_distance()
        #if percentage >= infothreshold:
            #infothreshold+=5.0
        s = ""
        writer.writerow([str(rospy.Time.now().to_sec()-start_time), str("%.2f" %(np.sum(distances))), str("%.2f" %percentage), str(distances)])

        rate.sleep()

    csvfile.close()





        