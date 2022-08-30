#!/usr/bin/env python3

import rospy, sys
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid


class Exploration_Tracker():
    
    def __init__(self, argv, argc):
        rospy.init_node("Exploration_Tracker", anonymous=True)
        print(argv)
        worldname = argv
        self.exploring_size = 0
        self.explored_cells = 0
        if worldname == "belgioioso":
            self.exploring_size = 68245
        elif worldname == "office":
            self.exploring_size = 86729
        elif worldname == "building":
            self.exploring_size = 240000


        rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.exploration_pub = rospy.Publisher('exploration_percentage', Float32, queue_size = 10)


    def callback_map(self, msg):
        data = np.array(msg.data)
        explored_cells = data >= 0
        self.explored_cells = len(data[explored_cells])
        self.calculate_explored_percentage()

    def calculate_explored_percentage(self):
        percentage = Float32()
        percentage.data = (self.explored_cells*100.0)/self.exploring_size
        self.exploration_pub.publish(percentage)
        print(percentage)


def main(argv, argc):
    explor_tracker = Exploration_Tracker(argv, argc)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1], len(sys.argv))
