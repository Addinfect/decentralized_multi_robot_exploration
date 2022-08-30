#!/usr/bin/env python3


import rospy, math, csv, os, datetime
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose

rospy.init_node('TestTracker', anonymous=True)


class InformationTracker:
    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.traveled_distance = 0.0
        self.old_x = 0.0
        self.old_y = 0.0
        self.map_free_cells = 0

    def callback_odometry(self, msg:Odometry):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.x
        distance_x = new_x-self.old_x
        distance_y = new_y-self.old_y
        self.traveled_distance += math.sqrt(distance_x**2+distance_y**2)
        self.old_x = new_x
        self.old_y = new_y

    def callback_map(self, msg:OccupancyGrid):
        # count all free grid points
        data = np.array(msg.data)
        free_space_cells = data == 0
        self.map_free_cells = len(data[free_space_cells])

    def get_traveled_distance(self) -> float:
        return self.traveled_distance

    def get_free_cells(self) -> int:
        return self.map_free_cells

tracker = InformationTracker()

rate = rospy.Rate(0.2)
date_time = (datetime.datetime.now().strftime('%d-%m-%Y-%H-%M-%S'))
rospy.loginfo_once("HELLOWWWW")
with open('data%s.csv' %date_time, 'w+') as csvfile:
    writer = csv.writer(csvfile, dialect='excel', delimiter=' ',
                            quotechar=';', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['Time'] + ['Traveled_Distance'] + ['Free Cells'])

    rospy.loginfo_once("TestTrackerPath: "+str(os.path.dirname(__file__)))
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        writer.writerow([str(rospy.Time.now().to_sec()-start_time), str(tracker.get_traveled_distance()), str(tracker.get_free_cells())])
        rate.sleep()

    csvfile.close()





        