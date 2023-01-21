#!/usr/bin/env python
# from PIL import Image
import cv2
import time
from datetime import datetime
import rospy
import sys
import copy
import time
import numpy as np
import tf
import os
from os.path import exists
from geometry_msgs.msg import Twist
from jackal_msgs.msg import Feedback, Status
from sensor_msgs.msg import NavSatFix, MagneticField, LaserScan, Imu, Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import io


#from spikeWaveEprop import spike_wave, get_gps_waypoints, update_weights, levy_flight, get_distance, gpsLat, gpsLon
#from config_aldrich import gpsLat_small, gpsLon_small, cost_map_small, num_costmaps

class JackalController:
	"""
	Script to record all current sensor data. Should be run from within Jackal.
	"""
	def __init__(self):
		#Publishing rate
		self.rate = rospy.Rate(10)

		#GPS Members
		self.gps_readings = []
		self.latitude = None
		self.longitude = None

		self.createGpsListener()
		self.createCameraListener()

		self.bridge = CvBridge()
                # @TODO remove hardcoded camera size, ok assumption for jackal...

                start_time = datetime.now() # current date and time
                dir_name = "recordings/%s/" % start_time.strftime("%m_%d_%Y%_H_%M_%S")
                os.makedirs(dir_name)
		self.cv2_vid = cv2.VideoWriter(dir_name + 'video.avi', cv2.VideoWriter_fourcc(*'DIVX'), 25, (1536, 1024))
                self.frame_log = open(dir_name + "frame_log.txt", "w")
                self.gps_log = open(dir_name + "gps_log.txt", "w")


	def createGpsListener(self):
		"""
		Creates node that subscribes to the Jackal GPS topic
		"""
		self.gpsnode = rospy.Subscriber('novatel/fix/', NavSatFix, self.updateGps)

	def updateGps(self, data):
		"""
		Callback to handle GPS data upon arrival
		"""
                print("GPS read")
		self.longitude = data.longitude
		self.latitude = data.latitude
		self.altitude = data.altitude
		# self.gps_readings.append([data.header.stamp.secs, self.longitude, self.latitude, self.altitude])
                self.gps_log.write("time: %r, lat: %r, long: %r, alt: %r\n" %(data.header.stamp.secs, data.latitude, data.longitude, data.altitude))

	def createCameraListener(self):
		"""
		Creates node that subscribes to the Jackal GPS topic
		"""
		self.camera = rospy.Subscriber('camera/image_raw/', Image, self.updateCamera)

	def updateCamera(self, data):
		"""
		Callback to handle Camera data upon arrival
		"""
                print("Grabed frame")
		timestamp = data.header.stamp.secs
		img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
                self.cv2_vid.write(img)
                self.frame_log.write(str(timestamp) + "\n")


	def saveGPS(self):
		"""
		Saves all GPS data into csv
		"""
		t = time.localtime()
		timestamp = time.strftime("%b-%d-%Y_%H%M%S", t)
		np.savetxt("gps-" + timestamp + '.csv', self.gps_readings, delimiter=',')
		np.savetxt("costdata-" + timestamp + '.csv', self.pathdata, delimiter=',')


	def unregisterAll(self):
		"""
		Stops all nodes. Saves GPS.
		"""
		self.gpsnode.unregister()
                self.cv2_vid.release()
		self.frame_log.close()
		self.gps_log.close()

	def awaitReadings(self):
		"""
		Pauses execution until readings from all sources have been recieved
		"""

		print("Waiting to read GPS data...")
		while self.longitude == None and controller.latitude == None:
			time.sleep(1)
		print("Done")



if __name__ == '__main__':
	try:
		rospy.init_node('controller', anonymous=True)
		controller = JackalController()
		# controller.awaitReadings()
		while True:
			try:
				time.sleep(.1)
			except KeyboardInterrupt:
				print("interuppted")
		    		break
		controller.unregisterAll()

	except rospy.ROSInterruptException:
		pass
