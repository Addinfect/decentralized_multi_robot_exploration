#!/usr/bin/env python3
from ast import Pass
import rospy
import sys
import tf
import time
import numpy as np
import rosservice
from copy import copy
from math import exp, hypot
from hungarian import Hungarian
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from functions import robot, informationGain
from std_msgs.msg import ColorRGBA
from frontier_exploration.msg import PointArray, RobotPosGoal, AuctionInt, AuctionFrontier, AuctionBids
from frontier_exploration.srv import RobotService
from visualization_msgs.msg import Marker

class AuctionAssigner(object):

	def __init__(self):
		# Initialization
		self.map_topic = rospy.get_param('~map_topic', '/map')
		self.info_radius = rospy.get_param('~info_radius', 0.2)							
		self.frontiers_topic = rospy.get_param('~frontiers_topic','/filtered_frontiers')
		self.n_robots = rospy.get_param('~n_robots')
		self.delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.2)
		self.rateHz = rospy.get_param('~rate', 1)
		self.robot_number = rospy.get_param('~robot_number')
		self.rate = rospy.Rate(self.rateHz)

		self.frontiers = PointArray()		
		# Subscribers 
		rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
		rospy.Subscriber(self.frontiers_topic, PointArray, self.frontier_callback)
	
		# Initialization
		self.mapData = OccupancyGrid()
		self.move_base_error_point = {}
		# Current goals for all robots
		self.current_goals = {}
		
		self.wait_for_map()
		self.wait_for_frontiers()
		 
		# Get the name of the robot and creat robots
		self.append_robot_name()
		# Rosservice definition and wait for availability
		self.mission_service = rospy.Service('startmission' +
			 str(self.robot_number), RobotService, self.handleStartmission)
		
		self.wait_for_all_services()
		print ("Services are ready!")

		#Auctions variables
		self.spare_goals = []
		self.auctioneer_id = -1
		self.auction_is_live = False
		self.auction_points = []
		self.bids = np.zeros((1,self.n_robots))
		self.auction_other_robot_stole_auction = False
		self.bids_initialized = False
		self.received_bids_count = 0

		rospy.Subscriber("auction/start", AuctionFrontier, self.auction_start_callback)
		rospy.Subscriber("auction/bids", AuctionBids, self.auction_bids_callback)
		rospy.Subscriber("auction/results", AuctionInt, self.auction_results_callback)
		self.pub_start_auction = rospy.Publisher("auction/start", AuctionFrontier, queue_size=10)
		self.pub_bids = rospy.Publisher("auction/bids", AuctionBids, queue_size=10)
		self.pub_results = rospy.Publisher("auction/results", AuctionInt, queue_size=10)
		self.pub_spare_goals = rospy.Publisher("robot_"+ str(self.robot_number) + "/spare_goals", Marker, queue_size=10)


	def map_callback(self, data):
		self.mapData = data

	def frontier_callback(self, data):
		# PointArray
		self.frontiers = data

	def auction_results_callback(self, msg:AuctionInt):
		#self.spare_goals.clear() #only acutal goals are in spare
		# add all Points where i won to the spare goal
		rospy.loginfo("Receive Result MSG my Robot_ID:" +str(self.robot_number) + str(msg))
		if not len(self.auction_points):
			raise ValueError('Empty List in auction_points in Results! Robot_ID:' +str(self.robot_number))
		for id, point in zip(msg.int_array.data,self.auction_points):
			if id == self.robot_number:
				self.spare_goals.append(point)
		"""
		goals = Marker()
		goals.header.frame_id = "map"
		goals.header.stamp = rospy.Time.now()
		goals.type = 8
		goals.scale = Vector3()
		goals.scale.x = 0.15
		goals.scale.y = 0.15
		goals.color = ColorRGBA()
		goals.color.r = 1.0
		goals.color.a = 1.0
		goals.points = self.spare_goals
		self.pub_spare_goals.publish(self.spare_goals)
		"""
		self.auction_is_live = False	#Auction finished
		self.auctioneer_id = -1

	def auction_bids_callback(self, msg:AuctionBids):
		rospy.loginfo("Receive Bid MSG my Robot_ID:" +str(self.robot_number) + str(msg))
		if self.auction_is_live and msg.receiver_id.data == self.robot_number:
			if not self.bids_initialized:
				l = len(msg.bids_array.data)
				self.bids = [0]*self.n_robots
				self.bids_initialized = True
			#saves all bids from other robots
			self.bids[msg.trasmittor_id.data] = list(msg.bids_array.data)
			self.received_bids_count += 1



	def auction_start_callback(self, msg:AuctionFrontier):
		rospy.loginfo("Receive Start MSG my Robot_ID:" +str(self.robot_number) + str(msg))
		if self.auction_is_live:
			#check lowest auctioneer number
			if msg.trasmittor_id.data < self.auctioneer_id:
				# change auctioneer
				if self.auctioneer_id == self.robot_number:
					self.auction_other_robot_stole_auction = True
			elif msg.trasmittor_id.data == self.robot_number:
				pass
			else:
				return	# new id is higher --> ignore is
		self.auctioneer_id = msg.trasmittor_id.data
		self.auction_is_live = True
		self.auction_points = msg.frontier_points
		self.bid_on_points_alternative()

	def bid_on_points(self):
		bid_msg = AuctionBids()
		bid_msg.trasmittor_id.data = self.robot_number
		bid_msg.receiver_id.data = self.auctioneer_id
		bid_msg.bids_array.data = [0] * len(self.auction_points)
		cost = []

		if len(self.spare_goals) < 2:
			for point in self.auction_points:
				cal_cost = self.calculate_cost(point,self.robots[self.robot_number].getPosition())
				#if cal_cost >= 3:
				cost.append(cal_cost)
				#else:
					#cost.append(99999999999)

			min_index = np.argmin(cost)
			cost[min_index] = 999999999999
			sec_min_index = np.argmin(cost)

			bid_msg.bids_array.data[min_index] = 10
			bid_msg.bids_array.data[sec_min_index] = 5

		self.pub_bids.publish(bid_msg)

	def bid_on_points_alternative(self):

		bid_msg = AuctionBids()
		bid_msg.trasmittor_id.data = self.robot_number
		bid_msg.receiver_id.data = self.auctioneer_id
		bid_msg.bids_array.data = [0] * len(self.auction_points)
		number_bids = 2
		if len(self.auction_points) < number_bids:
			number_bids = len(self.auction_points)
		if len(self.spare_goals) < number_bids:
			points = PointArray()
			points.points = self.auction_points
			list_of_utilities = self.get_list_of_utilities(points)
			response = self.start_new_mission(self.robot_number)
			list_of_sum_costs_and_frontier_occ = \
				self.get_list_of_sum_costs_and_frontier_occ(points, response.response)
			# Fill the current_goals dict
			self.current_goals[self.robot_number] = response.response.current_goal
			# The weight = cost - utility + frontier_occ
			cost =[a - b \
				for a, b in zip(list_of_sum_costs_and_frontier_occ, list_of_utilities)]

			
			bid_value = 100
			for i in range(number_bids):
				min_index = np.argmin(cost)
				cost[min_index] = 999999999999
				bid_msg.bids_array.data[min_index] = int(bid_value/(i+1))
				pass
			
			#sec_min_index = np.argmin(cost)

			
			#bid_msg.bids_array.data[sec_min_index] = 5

		self.pub_bids.publish(bid_msg)
		


	def wait_for_frontiers(self):
		# Wait if frontiers are not received yet
		while len(self.frontiers.points) < self.n_robots:
			pass

	def wait_for_map(self):
		# Wait if map is not received yet
		print ("Waiting for map")
		while len(self.mapData.data) < 1:
			pass
		print ("Map received.")

	def append_robot_name(self):
		self.robots = []
		for i in range (self.n_robots):
			self.robots.append(robot('/robot_' + str(i)))
		self.robots[self.robot_number].sendGoal(
				self.robots[self.robot_number].getPosition())

	def wait_for_all_services(self):
		print("Waiting for services...")
		for i in range (0, self.n_robots):
			rospy.wait_for_service('startmission' + str(i))
		
	def start_new_mission(self, robot_id):
		# Call the service_robot_id to get robot(robot_id) position
		service_response = RobotPosGoal()
		try:				
			start_mission = rospy.ServiceProxy(
				'startmission' + str(robot_id), RobotService)
			service_response = start_mission(robot_id)
			return service_response
		except rospy.ServiceException as exc:
			rospy.logwarn("Service did not process request: " + str(exc))	

	def handleStartmission(self, request):
		# request --> robot_id int32 type
		# response --> position Point() type
		response = RobotPosGoal()
		response.position = self.robots[request.robot_id].getPosition()
		response.current_goal = self.robots[request.robot_id].current_goal
		return response

	def calculate_weight_matrix(self, frontier_points):
		# Calculate the weight for every frontier point	and for all robots
		# Create matrix of weights (n x m)
		number_of_frontiers = len(frontier_points.points)
		weight_matrix = np.zeros((self.n_robots, number_of_frontiers))
						
		list_of_utilities = self.get_list_of_utilities(frontier_points)
		for i in range(0, self.n_robots):
			# Call the services to get position and current_goal of the robot
			response = self.start_new_mission(i)
			list_of_sum_costs_and_frontier_occ = \
				self.get_list_of_sum_costs_and_frontier_occ(frontier_points, response.response)
			# Fill the current_goals dict
			self.current_goals[i] = response.response.current_goal
			# The weight = cost - utility + frontier_occ
			weight_matrix[i] =[a - b \
				for a, b in zip(list_of_sum_costs_and_frontier_occ, list_of_utilities)]
			# Set all negative values to zero
			weight_matrix[i] = [0 if x < 0 else x for x in weight_matrix[i]]
		return weight_matrix
		
	def calculate_cost(self, frontier_point, robot_position):
		# Cost function for a single point = Euclidean distance
		lambda_c = 1.5
		cost = lambda_c * self.Euclidean_distance(
			frontier_point, robot_position)
		return cost

	def calculate_utility(self, frontier_point):
		# Utility function for a single point
		lambda_u = 2.0
		utility = lambda_u * (informationGain(self.mapData, 
			[frontier_point.x, frontier_point.y], self.info_radius))
		return utility

	def calculate_frontier_occupancy(self, frontier_point, current_goal):
		# Calculate frontier_occupancy_function for other mobile robots
		# And only if the frontier point is in range radius_front_occ from current_goal
		radius_front_occ = 8.0
		lambda_f = 20.0
		# distance = distance from frontier_point to current_goal
		distance = self.Euclidean_distance(frontier_point, current_goal)
		if (distance <= radius_front_occ):
			first_factor = \
				((frontier_point.x - current_goal.x) ** 2) / (2 * ((radius_front_occ) ** 2))
			second_factor = \
				((frontier_point.y - current_goal.y) ** 2) / (2 * ((radius_front_occ) ** 2))
			
			F = lambda_f * (exp(-(first_factor + second_factor)))
		else:
			F = 0
		return F
	
	def get_list_of_utilities(self, frontier_points):
		# Collect utilities of all frontier points
		list_of_utilities = []
		for point in frontier_points.points:
			list_of_utilities.append(self.calculate_utility(point))
		return list_of_utilities

	def get_list_of_sum_costs_and_frontier_occ(self, frontier_points, response):
		list_of_costs = []
		list_of_frontier_occ = []
		
		for point in frontier_points.points:
			# Cost function			
			list_of_costs.append(
				self.calculate_cost(point, response.position))
			# Frontier occupancy function
			list_of_frontier_occ.append(
				self.calculate_frontier_occupancy(point, response.current_goal))
		# Return the sum of these two lists
		list_of_sum_costs_and_frontier_occ = [a + b \
			for a, b in zip(list_of_costs, list_of_frontier_occ)]
		return list_of_sum_costs_and_frontier_occ

	def calculate_Hungarian(self, weight_matrix):
		# Find the minimum using Hungarian algorithm
		
		# If number of robots > number of frontiers
		# If the number of raws > number of columns in weight_matrix
		# if len(weight_matrix) > len(weight_matrix[0]): add!!
		
		# Check if the weight matrix is complete	
		if len(weight_matrix) == self.n_robots:
			h = Hungarian(weight_matrix)
			h.calculate()
			result = h.get_results()
		else:
			result = []
			rospy.logwarn("Invalid input for Hungarian algorithm.")
		print ('result je', result)
		
		return result

	def remove_points(self, mutable_centroids):
		# Remove current_goals and error points form constant_frontiers	
		constant_frontiers = PointArray()
		if len(self.current_goals) > 1 or len(self.move_base_error_point) > 0:
			current_goals_list = [*self.current_goals.values()]
			error_point_list = [*self.move_base_error_point.values()]
			# Save the points in constant frontiers
			for point in mutable_centroids.points:
				if not (self.check_if_point_in_list(current_goals_list, point) or \
					self.check_if_point_in_list(error_point_list, point)):							
					constant_frontiers.points.append(point)				
		else:
			for point in mutable_centroids.points:
				constant_frontiers.points.append(point)
		return constant_frontiers

	def check_if_point_in_list(self, list_of_points, point):
		for point_in_list in list_of_points:
			if (point.x == point_in_list.x and \
				point.y == point_in_list.y):
				return True
		return False 
	
	def Euclidean_distance(self, first_point, second_point):
		return hypot(first_point.x - second_point.x, 
			first_point.y - second_point.y)

	def run(self):

		rospy.loginfo("Robot assigner started")

		while not rospy.is_shutdown():
			self.wait_for_frontiers()
			mutable_centroids = copy(self.frontiers)
		
			# Remove current_goals and error points form constant_frontiers	
			constant_frontiers = self.remove_points(mutable_centroids)

			while not len(self.spare_goals):
				rospy.loginfo("########################### Wait for goals ID:"+ str(self.robot_number))
				if not self.auction_is_live:
				#publish winner & results
					self.auction_is_live = True
					start_msg = AuctionFrontier()
					start_msg.frontier_points = constant_frontiers.points
					start_msg.trasmittor_id.data = self.robot_number
					self.received_bids_count = 0
					self.pub_start_auction.publish(start_msg)
					self.auctioneer_id = self.robot_number
					
					#wait for bids
					rospy.sleep(1.0)
					start_time = rospy.Time.now()
					while(self.received_bids_count < self.n_robots):
						rospy.loginfo("Robot_%d: Wait for Bids! Received only: %d" %(self.robot_number, self.received_bids_count))
						if self.auction_other_robot_stole_auction:
							break
						if start_time+rospy.Duration(10) < rospy.Time.now():
							self.auction_is_live = False
							break
						rospy.sleep(0.2)
						pass

					#check all received bids
					if not self.auction_other_robot_stole_auction:
						results = list()
						rospy.loginfo("BIDS: \n \n \n" + str(self.bids))
						for i in range(len(self.bids[0])):		#len of bided points
							max_cost = 0
							max_index = -1
							for j in range(len(self.bids)):
								if self.bids[j][i] > max_cost:
									max_index = j
							results.append(max_index)

						result_msg = AuctionInt()
						result_msg.trasmittor_id.data = self.robot_number
						result_msg.int_array.data = results

						self.pub_results.publish(result_msg)
					#auction finished
					self.bids = np.zeros((1,self.n_robots))
					self.auction_other_robot_stole_auction = False
					self.bids_initialized = False
				rospy.sleep(self.delay_after_assignement)

			
			
			# set start and goal position
			robot_index = self.robot_number
			start_position = self.robots[self.robot_number].getPosition()
			goal_position = self.spare_goals.pop(0)	# fist goal assigned and removed

			
			# Send robot to the goal
			self.robots[self.robot_number].sendGoal(goal_position)
			rospy.loginfo("Robot_" + str(self.robot_number) +
				 "  assigned to point  " + str(goal_position))
			rospy.sleep(self.delay_after_assignement)
			
			start_time = time.time()
			start_time_timer = time.time()
			# Move until goal is reached, less that 1 meter
			while not (self.Euclidean_distance(
				start_position, goal_position)) < 1.0:
				
				# If the goal is aborted
				if (self.robots[self.robot_number].getState() == 4):
					print ("STATE", self.robots[self.robot_number].getState())
					# Rememeber the point to delete in the next iteration
					self.move_base_error_point[robot_index] = goal_position
					break

				# If a robot is in the same position for 5 seconds	
				current_position = self.robots[self.robot_number].getPosition()
				if (self.Euclidean_distance(
					start_position, current_position) < 0.2 and 
					(time.time() - start_time) > 4.0): 
					# Robot in the same position for 4 seconds
					print ('Robot ', self.robot_number, 'is not moving.')
					# Rememeber the point to delete in the next iteration
					self.move_base_error_point[robot_index] = goal_position
					break
				else:
					start_time = time.time()
					start_position = self.robots[self.robot_number].getPosition()

				# Generally, if the robot needs more than 10 seconds to 
				# reach the goal, delete that goal
				if (time.time() - start_time_timer) > 30.0:
					self.move_base_error_point[robot_index] = goal_position
					print ("MORE THAN 60 SECONDS!")
					break
				"""
				Geht nicht, da die Frontiers nicht genauch gleich sind
				Check if the Goal is unexplored and in the actual frontier list
				mutable_centroids = copy(self.frontiers)
				Remove current_goals and error points form constant_frontiers	
				#constant_frontiers = self.remove_points(mutable_centroids)
				if not goal_position in constant_frontiers.points:
					self.move_base_error_point[robot_index] = goal_position
					print ("GOAL WAS EXPLORED")
					break
				"""
				rospy.sleep(1.0)
			print ('Robot ', self.robot_number, 'reached the goal')		
			self.rate.sleep()


			
if __name__ == '__main__':
	rospy.init_node('Robot_assigner')
	try:
		robot_assigner = AuctionAssigner()
		rospy.sleep(2.0)
		robot_assigner.run()
	except rospy.ROSInterruptException:
		pass




		
