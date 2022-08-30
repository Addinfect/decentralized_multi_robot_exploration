#!/usr/bin/env python3

import rospy, sys, math
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry


class Exploration_Tracker():
    
    def __init__(self, argv, argc):
        rospy.init_node("Exploration_Tracker", anonymous=True)
        print(argv)
        worldname = argv[1]
        self.num_Robots = int(argv[2])
        self.exploring_size = 0
        self.explored_cells = 0
        if worldname == "belgioioso":
            self.exploring_size = 68245
        elif worldname == "office":
            self.exploring_size = 86729
        elif worldname == "building":
            self.exploring_size = 240000


        rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.traveled_distance = Float32MultiArray()
        #self.traveled_distance.layout.dim = (self.num_Robots,1)
        self.traveled_distance.data = []
        self.old_points = []    
        for i in range(self.num_Robots):
            rospy.Subscriber('/robot_'+str(i)+'/odom', Odometry, self.callback_odo, callback_args=i)
            self.traveled_distance.data.append(0.0)
            self.old_points.append((0.0,0.0))
        self.exploration_pub = rospy.Publisher('exploration_percentage', Float32, queue_size = 10)
        self.distance_pub = rospy.Publisher('traveled_distance', Float32MultiArray, queue_size=10)

    def callback_map(self, msg):
        data = np.array(msg.data)
        explored_cells = data >= 0
        self.explored_cells = len(data[explored_cells])
        self.calculate_explored_percentage()
        self.publish_distance()
    
    def callback_odo(self, msg, number):
        (old_x, old_y) = self.old_points[number]
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.x
        distance_x = new_x-old_x
        distance_y = new_y-old_y
        self.traveled_distance.data[number] += math.sqrt(distance_x**2+distance_y**2)
        self.old_points[number] = (new_x, new_y)
    def publish_distance(self):
        self.distance_pub.publish(self.traveled_distance)
        print(self.traveled_distance)

    def calculate_explored_percentage(self):
        percentage = Float32()
        percentage.data = (self.explored_cells*100.0)/self.exploring_size
        self.exploration_pub.publish(percentage)
        rospy.loginfo("Map Exploration " + str(percentage.data) + "%")
        print(percentage)


def main(argv, argc):
    explor_tracker = Exploration_Tracker(argv, argc)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))
