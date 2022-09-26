#!/usr/bin/env python
import rospy
import tf
import sys
import copy
import time
import numpy as np
from os.path import exists
from geometry_msgs.msg import Twist
from jackal_msgs.msg import Feedback, Status
from sensor_msgs.msg import NavSatFix, MagneticField, LaserScan, Imu

from spikeWaveEprop import spike_wave, get_gps_waypoints, update_weights, levy_flight, get_distance, gpsLat, gpsLon
from mapGenerator import gpsLat_small, gpsLon_small, cost_map_small

class JackalController:

	"""
	Script to record all current sensor data. Should be run from within Jackal.
	"""
	def __init__(self):
		#Publishing rate
		self.rate = rospy.Rate(10)

		#Driving members
		self.heading = None
		self.headingoffset = 0.0
		self.drivespeed = 0.5
		self.turnspeed = 0.15 # 0.075

		#Testing
		self.initialdistance = None
		self.maxturnspeed = 0.07
		self.minturnspeed = 0.03

		#Lidar Members
		self.scan = None
		self.obstaclepenalty = 0
		self.blockthreshold = 0.9
		self.consecutivethreshold = 15

		#Current Members
		self.current = None
		self.numcurrentread = 0
		self.totalcurrentread = 0

		#GPS Members
		self.gps_readings = []
		self.latitude = None
		self.longitude = None

		#Magnetometer Members
		self.heading = None

		self.createImuListener()
		self.createCurrentListener()
		self.createMagListener()
		self.createGpsListener()
		self.createDriver()


	"""Imu methods"""
	def createImuListener(self):
		"""
		Creates node that subscribes to the Jackal Imu topic
		"""
		self.imunode = rospy.Subscriber('scan', LaserScan, self.updateImu)

	def updateImu(self, data):
		"""
		Callback to handle IMU data upon arrival
		"""
		self.scan = data.ranges[170:370]

	def checkForObstacle(self):
		consecutive = 0
		for i, val in enumerate(self.scan):
			if val > 0.1 and val < 2:
				consecutive += 1
			else:
				consecutive = 0

			if consecutive > self.consecutivethreshold:
				return True

		return False


	def createCurrentListener(self):
		"""
		Creates node that subscribes to the Jackal Status topic
		"""
		self.currentnode = rospy.Subscriber('feedback', Feedback, self.updateCurrent)

	def updateCurrent(self, data):
		"""
		Callback to handle IMU data upon arrival
		"""
		#self.current = data.drive_current
		self.current = min(data.drivers[0].current, data.drivers[1].current)
		self.numcurrentread += 1

		self.totalcurrentread += self.current

	def resetCostTracker(self):
		"""
		Sets total current read and number of current readings back to 0. Call when entering new waypoint to get new average.
		"""
		self.numcurrentread = 0
		self.totalcurrentread = 0
		self.obstaclepenalty = 0

	"""Magnetometer methods"""

	def createMagListener(self):
		"""
		Creates node that subscribes to the Jackal magnetometer topic
		"""
		self.magnode = rospy.Subscriber('imu/data', Imu, self.updateMag)

	def updateMag(self, data):
		"""
		Callback to handle magnetometer data upon arrival
		"""

		# quarternion format
		q = data.orientation
		_roll, _pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.heading = yaw
		self.heading *= -1
		self.heading += 2*np.pi
		if self.heading > 2*np.pi:
			self.heading = self.heading - 2*np.pi

	"""GPS Related methods"""

	def createGpsListener(self):
		"""
		Creates node that subscribes to the Jackal GPS topic
		"""
		self.gpsnode = rospy.Subscriber('navsat/fix/', NavSatFix, self.updateGps)

	def updateGps(self, data):
		"""
		Callback to handle GPS data upon arrival
		"""
		self.longitude = data.longitude
		self.latitude = data.latitude
		self.altitude = data.altitude
		self.gps_readings.append([data.header.stamp.secs, self.longitude, self.latitude, self.altitude])

	def saveGPS(self):
		"""
		Saves all GPS data into csv
		"""
		t = time.localtime()
		timestamp = time.strftime("%b-%d-%Y_%H%M%S", t)
		np.savetxt("gps-" + timestamp + '.csv', self.gps_readings, delimiter=',')

	"""Driving Related methods"""

	def createDriver(self):
		"""
		Creates node that publishes to the Jackal driving topic
		"""
		self.drivernode = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	def makeTwist(self, lx, ly, lz, ax, ay, az):
		"""
		Creates a "Twist" message readable by Jackal
			lx, ly, lz: linear speed in x, y, and z directions
			ax, ay, az: angular speed in x, y, and z directions
		"""
		msg = Twist()
		msg.linear.x = lx
		msg.linear.y = ly
		msg.linear.z = lz
		msg.angular.x = ax
		msg.angular.y = ay
		msg.angular.z = az
		return msg

	def buildTwist(self, angle, vel):
		"""
		Simplifies creation of "Twist" message to available directions
			angle: rotational velocity
			vel: linear velocity
		"""
		return self.makeTwist(vel, 0.0, 0.0, 0.0, 0.0, angle)

	def Drive(self, linear, rotational):
		"""
		Publishes driving message to Jackal
			linear: linear velocity
			rotational: rotational velocity
		"""
		msg = self.makeTwist(linear, 0, 0, 0, 0, rotational)
		self.drivernode.publish(msg)

	def getAngleDistance(self, start, end):
		"""
		Returns bearing from start coordinate to end coordinate
			start: (latitude, longitude) start GPS point
			end: (latitude, longitude) end GPS point
		"""
		ang = np.arctan2((end[1] - start[1]),(end[0] - start[0]))
		distance = np.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)

		if ang < 0:
			ang = ang + 2*np.pi
	
		return ang, distance

	def scaledSpeed(self, dist):
		"""
		Scales speed according to distance. Starts fast and slows down as Jackal gets closer to waypoint.
		"""
		return max(self.maxturnspeed * (dist/self.initialdistance), self.minturnspeed)

	def turnToWaypoint(self, latitude, longitude):
		"""
		Issues drive commands to turn Jackal to match the desired coordinates
			latitude, longitude: coordinates of destination
		"""
		ang, self.initialdistance = self.getAngleDistance((self.latitude, self.longitude),(latitude, longitude))

		while abs(self.heading - ang) > 0.05:
			if abs(self.heading - ang) < np.pi:
				if self.heading > ang:
					self.Drive(0, self.turnspeed * 2)
				else:
					self.Drive(0, -self.turnspeed * 2)
			else:
				if self.heading > ang:
					self.Drive(0, -self.turnspeed * 2)
				else:
					self.Drive(0, self.turnspeed * 2)
			self.rate.sleep()
			ang, _ = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))

		return

	def driveToward(self, angle, dist):
		"""
		Issues a single command toward the desired angle, returns distance 
		"""
		if abs(self.heading - angle) < np.pi:
			if self.heading > angle:
				self.Drive(self.drivespeed, self.scaledSpeed(dist))
			else:
				self.Drive(self.drivespeed, -self.scaledSpeed(dist))
		else:
			if self.heading > angle:
				self.Drive(self.drivespeed, -self.scaledSpeed(dist))
			else:
				self.Drive(self.drivespeed, self.scaledSpeed(dist))

		self.rate.sleep()

	def driveToWaypoint(self, latitude, longitude):
		"""
		Drives Jackal to the coordinates. Returns a tuple (bool, float). bool is true for successfully reaching false for unsuccessful, float is the cost.
			latitude, longitude: coordinates
		"""
		self.turnToWaypoint(latitude, longitude)

		#Start tracking cost after turning to direction
		self.resetCostTracker()
		pathcomplete = True

		ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))
		while dist > 0.000002:
			self.driveToward(ang, dist)

			#Recalculate bearing
			ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))
		cost = self.computeCost()
		return pathcomplete, cost

	def drivePath(self, start, end, weights, costmap):
		"""
		Drives Jackal from one waypoint to another using Spike Wave algorithm. Returns a map of the losses.
		"""

		#Run spike wave algorithm on initial map/weights
		et, path = spike_wave(weights, costmap, start[0], start[1], end[0], end[1])
		waypointpath = get_gps_waypoints(path, gpsLat_small, gpsLon_small)
		reversedpath = list(reversed(copy.deepcopy(path)))
		new_costmap = copy.deepcopy(costmap)

		print("Path:")
		print(reversedpath)

		for i in range(len(waypointpath)):
			latitude = float(waypointpath[i][0])
			longitude = float(waypointpath[i][1])

			print("Driving to waypoint (%d, %d) at (%f, %f)" % (reversedpath[i][0], reversedpath[i][1], latitude, longitude))
			completed, cost = controller.driveToWaypoint(latitude, longitude)
			controller.saveGPS()

			#Do not update the cost of going to the initial waypoint (Assumed to start at the first waypoint)
			if(i != 0):
				#Update cost on map of waypoint destination to observed values
				print("Computed cost for this path: %f" % cost)
				print("Updating at %d, %d" % (reversedpath[i][0], reversedpath[i][1]))
				new_costmap[reversedpath[i][0]][reversedpath[i][1]] = cost
				#If an obstacle was encountered back up to the previous waypoint and end this trial (later we will rerun)
				if(not completed):
					print("Obstacle encountered")
					controller.driveToWaypoint(waypointpath[i-1][0], waypointpath[i-1][1])
					#Return incomplete path
					return et, path[len(path) - i:-1], new_costmap

		return et, path, new_costmap


	def computeCost(self):
		"""
		Calculates the cost given current data (average readings for now)
		"""

		MAX_CURRENT = 3.5
		NORMALIZED_MIN = 1
		NORMALIZED_RANGED = 4

		#current_normalized = (((self.totalcurrentread / self.numcurrentread) / MAX_CURRENT) * NORMALIZED_RANGED) + NORMALIZED_MIN

		if(self.numcurrentread != 0):
			#return self.obstaclepenalty + current_normalized
			costTotal = self.totalcurrentread / self.numcurrentread
			return costTotal
			#return ((costTotal ** 2) / 3) * 5
		else:
			return 0

	def unregisterAll(self):
		"""
		Stops all nodes. Saves GPS.
		"""
		self.saveGPS()
		self.imunode.unregister()
		self.currentnode.unregister()
		self.gpsnode.unregister()
		self.drivernode.unregister()
		self.magnode.unregister()

	def awaitReadings(self):
		"""
		Pauses execution until readings from all sources have been recieved
		"""

		print("Waiting to read GPS data...")
		while self.longitude == None and controller.latitude == None:
			time.sleep(1)
		print("Done")

		print("Waiting to read Magnetometer data...")
		while self.heading == None:
			time.sleep(1)
		print("Done")

		#print("Waiting to read Lidar data...")
		#while self.scan == None:
		#	time.sleep(1)
		#print("Done")

		#print("Waiting to read Current data...")
		#while self.current == None:
		#	time.sleep(1)
		#print("Done")


if __name__ == '__main__':
	try:
		rospy.init_node('controller', anonymous=True)
		controller = JackalController()
		controller.awaitReadings()

		if(exists("cost_map.npy")):
			cost_map = np.load("cost_map.npy")
		else:
			#cost_map = np.array([[15, 15, 15, 15, 15, 15, 15, 15, 15, 15],
			#			[15, 1, 1, 1, 1, 1, 1, 1, 1, 15],
			#			[15, 1, 1, 1, 1, 1, 1, 1, 1, 15],
			#			[15, 1, 1, 1, 1, 1, 1, 1, 1, 15],
			#			[15, 1, 1, 1, 10, 10, 10, 1, 1, 15],
			#			[15, 1, 1, 1, 10, 10, 10, 1, 1, 15],
			#			[15, 1, 1, 1, 10, 10, 10, 1, 1, 15],
			#			[15, 1, 1, 1, 1, 1, 1, 1, 1, 15],
			#			[15, 1, 1, 1, 1, 1, 1, 1, 1, 15],
			#			[15, 15, 15, 15, 15, 15, 15, 15, 15, 15]]).astype('float64')
			
			cost_map = cost_map_small

		n1 = cost_map.shape[0]
		n2 = cost_map.shape[1]

		if(exists("wgt.npy")):
			wgt = np.load("wgt.npy")
		else:
			wgt = np.zeros([n1, n2, n1, n2])
			w_init = 1
			for i in range(n1):
				for j in range(n2):
					for m in range(-1, 2):
						for n in range(-1, 2):
							# 8 directions for these mazes
							if (i + m >= 0) and (i + m < n1) and (j + n >= 0) and (j + n < n2) and (m != 0 or n != 0):
								wgt[i][j][i + m][j + n] = w_init

		if(len(sys.argv) > 1 and sys.argv[1] == "levy"):
			# Run the spike wave navigation for N trials. Use a Levy Flight distribution to choose a waypoint.
			# The waypoint must be at least 1 grid position away from the current position.

			try:
				num_trials = float(sys.argv[2])
			except:
				print("Incorrect number of trials, defaulting to 1")
				num_trials = 1

			wp_end = np.array([2, 2])
			wp_start = np.copy(wp_end)
			t = 0
			trials = num_trials
			while t < trials:
				lf = levy_flight(2, 3)  # levy_flight will return a new waypoint
				# waypoint coordinates must be whole numbers
				wp_end[0] += round(lf[0])
				# check that the waypoint is within the map boundaries
				if wp_end[0] < 0:
					wp_end[0] = 0
				elif wp_end[0] >= n1:
					wp_end[0] = n1-1

				wp_end[1] += round(lf[1])
				if wp_end[1] < 0:
					wp_end[1] = 0
				elif wp_end[1] >= n1:
					wp_end[1] = n1-1

				print("Path %d from %d,%d to %d,%d" % (t+1, wp_start[0], wp_start[1], wp_end[0], wp_end[1]))

				# if the new waypoint is not over 1 grid position away from the current position,
				#     skip this waypoint and find another.
				if get_distance(wp_start, wp_end) > 1:
					t += 1
					et, p, cost_map = controller.drivePath((wp_start[0], wp_start[1]), (wp_end[0], wp_end[1]), wgt, cost_map)

					# set wp_end to the end of the path just in case path was not reached.
					wp_end = np.array([p[0][0], p[0][1]])
					wp_start = np.copy(wp_end)

					# assume that at this point the robot has traversed a path and updated a cost map
					wgt = update_weights(cost_map, et, p, wgt)

			# Calculate the average weight value projecting into each neuron.  This is useful for diagnostic purposes.
			# map_loss roughly corresponds to the loss and gives an idea of how much of the environment has been learned.
			map_loss = np.zeros([n1, n2])
			for i in range(n1):
				for j in range(n2):
					val = 0
					cnt = 0
					for m in range(n1):
						for n in range(n2):
							if wgt[i][j][m][n] > 0:
								val += wgt[i][j][m][n]
								cnt += 1
					map_loss[i][j] = val/cnt

			# Save the map_loss to a text file and the 4D weight array to a python readable file.
			np.savetxt("map_loss.txt", map_loss, fmt="%2.3f")
			np.save("wgt.npy", wgt)
			np.save("cost_map.npy", cost_map)
		elif(len(sys.argv) > 1 and sys.argv[1] == "spikewave"):
			startx = raw_input("Start x\n")
			starty = raw_input("Start y\n")
			endx = raw_input("Dest. x\n")
			endy = raw_input("Dest. y\n")
			et, p, cost_map = controller.drivePath((int(startx), int(starty)), (int(endx), int(endy)), wgt, cost_map)
		elif(len(sys.argv) > 1 and sys.argv[1] == "single"):
			#end_lat = input("Dest. latitude\n")
			#end_long = input("Dest. longitude\n")
			#complete, cost = controller.driveToWaypoint(float(end_lat), float(end_long))
			complete, cost = controller.driveToWaypoint(49.899893753325365, 8.900000762236186)
			# go back to origin/home
			complete, cost = controller.driveToWaypoint(49.90000031474116, 8.900000023288687)
			complete, cost = controller.driveToWaypoint(49.89999999222913, 8.899835414617245)
			complete, cost = controller.driveToWaypoint(49.90000031474116, 8.900000023288687)
			complete, cost = controller.driveToWaypoint(49.89999998537977, 8.900165028457785)
			complete, cost = controller.driveToWaypoint(49.90000031474116, 8.900000023288687)
			complete, cost = controller.driveToWaypoint(49.90010671959018, 8.900000030463996)
			complete, cost = controller.driveToWaypoint(49.90000031474116, 8.900000023288687)
			print("Total cost for this path: %f" % cost)

		controller.unregisterAll()


	except rospy.ROSInterruptException:
		pass
