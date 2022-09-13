#!/usr/bin/env python3

import rospy, sys, math, csv, os
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry


class Exploration_Tracker():
    
    def __init__(self, argv, argc):
        rospy.init_node("Exploration_Tracker", anonymous=True)
        print(argv)
        self.worldname = argv[1]
        self.num_Robots = int(argv[2])
        self.algorithmus = argv[3]
        self.exploring_size = 0
        self.explored_cells = 0
        if self.worldname == "belgioioso":
            self.exploring_size = 68245
        elif self.worldname == "office":
            self.exploring_size = 86729
        elif self.worldname == "building":
            self.exploring_size = 240000
        elif self.worldname == "big_office":
            self.exploring_size = 95500


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
        self.percentage = 0.0
        self.start_time = 0.0
        self.run_finished = False
        self.start_time_initialized = False
        self.row = []
        self.tracking_borders = [50,75,90,95,98]
        self.actual_tracking_index = 0


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
        if (distance_x+distance_y) > 0.1 and not self.start_time_initialized:   #Robot moved more than 0.1 meter
            self.start_time_initialized = True
            self.start_time = rospy.Time.now().to_sec()
    def publish_distance(self):
        self.distance_pub.publish(self.traveled_distance)
        print(self.traveled_distance)

    def calculate_explored_percentage(self):
        percentage = Float32()
        percentage.data = (self.explored_cells*100.0)/self.exploring_size
        self.exploration_pub.publish(percentage)
        self.percentage = percentage.data
        rospy.loginfo("Map Exploration " + str(percentage.data) + "%")
        rospy.loginfo("ROW:"+ str(self.row))
        print(percentage)

        if self.percentage >= self.tracking_borders[self.actual_tracking_index] and not self.run_finished:
            self.save_data_from_run(self.tracking_borders[self.actual_tracking_index])
            if(self.tracking_borders[self.actual_tracking_index] == self.tracking_borders[-1]):
                self.run_finished = True
            self.actual_tracking_index += 1
            

    def save_data_from_run(self, percentage:int):
        with open('1_results.csv', 'a+') as csvfile:
            writer = csv.writer(csvfile, dialect='excel', delimiter=' ',
                                    quotechar=';', quoting=csv.QUOTE_MINIMAL)
            """
            writer.writerow(['World'] + ['Number_Robots'] + ['Assigner'] + ['Percentage'] + ['Time'] + ['Total Distance'] +
             ['Distance Robot_0'] + ['Distance Robot_1'] + ['Distance Robot_2']
              + ['Distance Robot_3'] + ['Distance Robot_4'] + ['Distance Robot_5'])
            """
            rospy.loginfo_once("TestTrackerPath: "+str(os.path.dirname(__file__)))
            total_distance = sum(self.traveled_distance.data)
            row = [self.worldname, str(self.num_Robots), self.algorithmus, str(percentage), str(rospy.Time.now().to_sec()-self.start_time), str("%.2f"%total_distance), "0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
            for index, distance in enumerate(self.traveled_distance.data):
                row[6+index] = ("%.2f"%distance)
            
            self.row = row
            writer.writerow(row)


def main(argv, argc):
    explor_tracker = Exploration_Tracker(argv, argc)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main(sys.argv, len(sys.argv))
