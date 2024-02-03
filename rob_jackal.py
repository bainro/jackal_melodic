import os
import tf
import time
import math
import rospy
import pickle
import argparse
import subprocess
import numpy as np
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from placecell import PlaceNetwork
from utils import loadNetwork, saveNetwork, levy_flight, get_distance
from jackal_msgs.msg import Feedback
from geometry_msgs.msg import Twist, PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import LaserScan, Imu, MagneticField, NavSatFix

# Initialize argparser
parser = argparse.ArgumentParser(
                prog='Jackal Spikewave',
                description='Spiking Wavefront Propagation with Jackal')
parser.add_argument('--type', type=str, default='single', help='Specifies the test type (spikewave, single, test, or goals)')
parser.add_argument('--trials', type=int, default=10, help='Number of trials for spikewave test')
parser.add_argument('--rosbag', type=bool, default=False, help='Record rosbag (T/F)')
parser.add_argument('--list', type=str, default='waypoints.txt', help='List of waypoints')
parser.add_argument('--start', type=int, default=0, help='Starting waypoint')
parser.add_argument('--x', type=int, default=9 , help='Starting x')
parser.add_argument('--y', type=int, default=9, help='Starting y')
args = parser.parse_args()

class JackalController:
    """
    Class for creating listeners for jackal topics and issuing commands.
    """

    def __init__(self,
                 lidar = True,
                 lidartopic = 'scan',
                 status = True,
                 statustopic = 'feedback',
                 heading = True,
                 headingtopic = '/gx5/imu/data',
                 gps = True,
                 gpstopic = '/fone_gps/fix',
                 odom = True,
                 odomtopic = 'odometry/filtered',
                 wifi = True,
                 wifi_acc_topic = '/wifi_strength',
                 gps_acc_topic = '/fone_gps/acc'):
        
        # Creates publisher for making drive commands
        self.createSimpleDriver() 
        # Creates publisher for making goal commands
        self.createGoalPublisher() 
        self.createGoalStatusListener()
        # hard-coded since no behavior conditions on it yet
        self.createGPSaccListener(gps_acc_topic) 
        self.gps_acc = math.nan
        self.moving = False
        self.curstatus = GoalStatus.SUCCEEDED

        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(10)

        self.lidar = lidar
        self.status = status
        self.heading = heading
        self.gps = gps
        self.lidardata = None
        self.statusdata = None
        self.headingdata = None
        self.gpsdata = None
        # used to speedup calibration
        self.last_heading_i = 0
        self.has_calibrated = False

        #Obstacle Avoidance
        self.blocked = False
        self.blockdir = 0
        self.first = False

        #COST: CURRENT
        self.numcurrent = 0
        self.totalcurrent = 0

        #COST: OBSTACLE
        self.obspenalty = 0
        self.obstotal = 0

        #COST: GPSACC
        self.numgpsacc = 0
        self.totalgpsacc = 0

        #COST: WIFI
        self.wificounter = 0
        self.wifitotal = 0

        #COST: SLOPE
        self.slopecost = 0
        self.slopecounter = 0

        #NORMALIZING CONSTANTS. Max is two standard deviations from average
        self.CURRMIN = 0.6
        self.CURRMAX = 10.0
        self.GPSMIN = 3.79
        self.GPSMAX = 4.1
        self.WIFIMIN = 40
        self.WIFIMAX = 65
        self.SLOPEMIN = 0.0
        self.SLOPEMAX = 0.5

        if lidar:
            self.createLidarListener(lidartopic)
            self.lidartopic = lidartopic
        if status:
            self.createStatusListener(statustopic)
            self.statustopic = statustopic
        if heading:
            self.createHeadingListener(headingtopic)
            self.headingtopic = headingtopic
        if gps:
            self.createGpsListener(gpstopic)
            self.gpstopic = gpstopic
            self.gps_acc_topic = gps_acc_topic
        if odom:
            self.createOdomListener(odomtopic)
            self.odomtopic = odomtopic
        if wifi:
            self.createWifiListener(wifi_acc_topic)
            self.wifitopic = wifi_acc_topic

        # @TODO add "if slope" logic
        self.createSlopeListener('/gx5/imu/data')

    def createWifiListener(self, topic):
        self.wifinode = rospy.Subscriber(topic, Int32, self.wifiCallback)

    def wifiCallback(self, data):
        self.wifistrength = float(data.data)
        self.wificounter += 1
        self.wifitotal += self.wifistrength

    def createSlopeListener(self, topic):
        self.slopenode = rospy.Subscriber(topic, Imu, self.slopeCallback)
	
    def slopeCallback(self, m):
        x = m.orientation.x
        y = m.orientation.y
        z = m.orientation.z
        w = m.orientation.w
        roll, pitch, _yaw = tf.transformations.euler_from_quaternion([x, y, z, w])	
        # roll is +/-3.14 on flat gnd; pitch is +/- 0
        if roll > 0:
            roll -= 3.14
        else:
            roll += 3.14
        current_slope_cost = abs(roll) + abs(pitch)
            # print(f'current_slope_cost: {current_slope_cost}')
        self.slopecost += current_slope_cost
        self.slopecounter += 1

    def rosbag(self, record=True):
        # Records rosbag for offline analysis.
        # start recording
        if record:
            self.bag_file = f'{int(time.time())}' + '.bag'
            bash_cmd =  f'rosbag record -O {self.bag_file} {self.lidartopic} {self.statustopic} '
            bash_cmd += f' {self.headingtopic} {self.gpstopic} {self.odomtopic} {self.gps_acc_topic} '
            bash_cmd += f' /throttled_img /wifi_strength /gx5/imu/data &' # background
            subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
        else:
            # try stopping any already running rosbag recordings
            bash_cmd = f'kill $(pgrep -f "rosbag record -O ")'
            os.system(bash_cmd)

    def createSimpleDriver(self):
        """
        Creates publisher for making drive commands
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

    def resetCostTracker(self):
        """
        Sets total current read and number of current readings back to 0. Call when entering new waypoint to get new average.
        """
        #COST: CURRENT
        self.numcurrent = 0
        self.totalcurrent = 0

        #COST: OBSTACLE
        self.obspenalty = 0
        self.obstotal = 0

        #COST: GPSACC
        self.numgpsacc = 0
        self.totalgpsacc = 0

        #COST: WIFIACC
        self.wifitotal = 0
        self.wificounter = 0

        #COST: SLOPE
        self.slopecost = 0
        self.slopecounter = 0
        

    def normalizeCost(self, val, minval,  maxval):
        if val < minval:
            return 1
        elif val > maxval:
            return 10
        else:
            return 1 + 9 * ((val - minval) / (maxval - minval))

    def computeCost(self):
        if self.numcurrent != 0:
            currentcost = self.normalizeCost(self.totalcurrent / self.numcurrent, self.CURRMIN, self.CURRMAX)
        else: currentcost = -1
        if self.obstotal != 0:
            obs = self.normalizeCost(self.obspenalty / self.obstotal, 0, 1)
        else: obs = -1
        if self.numgpsacc != 0:
            gpsacc = self.normalizeCost(self.totalgpsacc / self.numgpsacc, self.GPSMIN, self.GPSMAX)
        else: gpsacc = -1
        if self.wificounter != 0:
            wifiacc = self.normalizeCost(self.wifitotal / self.wificounter, self.WIFIMIN, self.WIFIMAX)
        else: wifiacc = -1
        if self.slopecounter != 0:
            slopecost = self.normalizeCost(self.slopecost / self.slopecounter, self.SLOPEMIN, self.SLOPEMAX) #max(1, 20*(self.slopecost / self.slopecounter))
        else: slopecost = -1

        #TODO: Add other costs and normalizing stuff


        #Last is obstacle map. That will only be updated when obstacle is encountered and never here
        return [currentcost, obs, gpsacc, wifiacc, slopecost, -1]

    def driveToWaypoint(self, latitude, longitude, timeout=False):
        """
        Drives Jackal to the coordinates. Returns a tuple (bool, float). bool is true for successfully reaching false for unsuccessful, float is the cost.
        latitude, longitude: coordinates
        """
        #self.turnToWaypoint(latitude, longitude)

		#Start tracking cost after turning to direction
        self.resetCostTracker()

        if timeout:
            start_time = time.time()


        ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))
        while dist > 0.9:
            self.driveToward(ang)
            #print("Distance to waypoint " + str(dist) + " | Heading " + str(self.heading) + " Angle to bearing: " + str(ang))
            ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))

            if timeout and time.time() - start_time > 45:
                print("Could not reach waypoint in 45 seconds")
                return self.computeCost(), False

        return self.computeCost(), True

    def driveToward(self, angle):
        """
        Issues a single command toward the desired angle, returns distance 
        """
        padding = np.pi/12
        if self.blocked:
            self.Drive(0.0, self.blockdir * 0.5)
            self.obspenalty += 1
        else:
            if abs(self.heading - angle) < np.pi:
                if self.heading < angle - padding:
                    self.Drive(0.5, 0.75)
                elif self.heading > angle  + padding:					
                    self.Drive(0.5, -0.75)
                else:
                    self.Drive(0.5, 0)
            else:
                if self.heading < angle - padding:
                    self.Drive(0.5, -0.75)
                elif self.heading > angle  + padding:					
                    self.Drive(0.5, 0.75)
                else:
                    self.Drive(0.5, 0)

        self.obstotal += 1
        self.rate.sleep()

    def computeRawCosts(self):
        """
        Returns current, obstacle, and slope costs for testing RRT vs SpikeWave
        """
        return [self.totalcurrent, self.numcurrent, self.obspenalty, self.obstotal, self.slopecost, self.slopecounter]

    def drivePathTest(self, path, points):
        """
        Drives Jackal to path, return time, current, obstacle, and slope costs for testing RRT vs SpikeWave
        """

        totalcosts = []

        for i in range(len(path)):
            point = path[i]
            latitude = point[0]
            longitude = point[1]
            print("Driving to waypoint (%d, %d)" % (points[i][0], points[i][1]))

            self.turnToWaypoint(self.latitude, self.longitude, latitude, longitude)
            starttime = time.time()
            _, success = self.driveToWaypoint(latitude, longitude, True)
            endtime = time.time()

            costs = [endtime - starttime] + self.computeRawCosts()
            totalcosts.append(costs)

            if not success:
                print("Could not reach waypoint")
                return None, False

        return totalcosts, True

    def drivePath(self, path, network, points):
        """
		Drives Jackal to path, return costs.
		"""
        #print(path)
        costs = []
        for i in range(len(path)):
            #point = network.cells[network.points[(path[0], path[1])]].origin
            point = path[i]
            latitude = point[0]
            longitude = point[1]
            print("Driving to waypoint (%d, %d)" % (points[i][0], points[i][1]))
            self.turnToWaypoint(self.latitude, self.longitude, latitude, longitude)

            if self.first:
                cost, completed = self.driveToWaypoint(latitude, longitude, False)
                self.first = False
            else:
                cost, completed = self.driveToWaypoint(latitude, longitude, True)

            #Do not update initial waypoint
            if i != 0:
                if completed is False:
                    cost[-1] = 10.0 
                    print("Returning to previous waypoint")
                    #self.turnToWaypoint(self.latitude, self.longitude, network.cells[network.points[(7, 7)]].origin[0], network.cells[network.points[(7, 7)]].origin[1])
                    #_, _ = self.driveToWaypoint(network.cells[network.points[(7, 7)]].origin[0], network.cells[network.points[(7, 7)]].origin[1], False)


                    self.turnToWaypoint(self.latitude, self.longitude, path[i-1][0], path[i-1][1])
                    _, _ = self.driveToWaypoint(path[i-1][0], path[i-1][1], False)
                    i = i-1
                else:
                    #Update visitation count for current cell
                    network.cells[network.points[points[i]]].visitation += 1

                print("Computed cost for this path:")
                print(cost)
                costs.append(cost)
                if completed is False:
                    return costs, i

        return costs, i

    ###MOVE BASE####
    def createGoalPublisher(self):
        """
        Creates publisher for making goal commands
        """
        self.goalnode = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    def publishGoal(self, x, y):
        """
        Creates a "PoseStamped" message readable by Jackal
            x, y, z: position in x, y, and z directions
            qx, qy, qz, qw: quaternion for orientation
        """

        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = self.odomdata.pose.pose.position.z
        
        msg.pose.orientation.x = 0#self.odomdata.pose.pose.orientation.x
        msg.pose.orientation.y = 0#self.odomdata.pose.pose.orientation.y
        msg.pose.orientation.z = 1#np.abs(self.odomdata.pose.pose.orientation.z)
        msg.pose.orientation.w = 0#self.odomdata.pose.pose.orientation.w
        self.goalnode.publish(msg)

    def moveToGoal(self, startlat, startlong, endlat, endlong):
        d = self.turnToWaypoint(startlat, startlong, endlat, endlong)

        # Wait a bit for the heading to stabilize
        time.sleep(3)
        x = self.odomdata.pose.pose.position.x + d*np.cos(self.odomheading)
        y = self.odomdata.pose.pose.position.y + d*np.sin(self.odomheading)

        self.publishGoal(x, y)

    def createGoalStatusListener(self):
        self.goalstatus = rospy.Subscriber("/move_base/status", GoalStatusArray, self.updateGoalStatus)

    def updateGoalStatus(self, data):
        if len(data.status_list) > 0:
            goalstatus = data.status_list[-1].status
        else:
            return
        
        if self.curstatus == GoalStatus.SUCCEEDED and goalstatus == GoalStatus.ACTIVE:
            print("Moving to waypoint")
            self.moving = True
            self.curstatus = goalstatus
        elif self.curstatus == GoalStatus.ACTIVE and goalstatus == GoalStatus.SUCCEEDED:
            print("Arrived at waypoint")
            self.moving = False
            self.curstatus = goalstatus
            

    ### LIDAR ###
    def createLidarListener(self, topic):
        """
        Subscriber to Jackal Lidar topic
        """
        self.lidarnode = rospy.Subscriber(topic, LaserScan, self.updateLidar)

    def updateLidar(self, data):
        """
        Handles Lidar data
        """
        self.lidardata = data
        self.scan = data.ranges[170:370]
        self.checkForObstacle()

    def checkForObstacle(self):	
        consecutive = 0
        points = []
        for i, val in enumerate(self.scan):
            if val > 0.1 and val < 2:
                consecutive += 1
                points.append(i)
            else:
                consecutive = 0
                points = []

            if consecutive > 15:
                self.blocked = True

                #Turn right if the midpoint of the obstacle is on the left side
                if points[int(len(points) / 2)] < len(self.scan) / 2:
                    self.blockdir = -1
                else:
                    self.blockdir = 1
                return

        self.blocked = False

    ### STATUS ###
    def createStatusListener(self, topic):
        """
        Subscriber to Jackalstr Status topic
        """
        self.statusnode = rospy.Subscriber(topic, Feedback, self.updateStatus)

    def updateStatus(self, data):
        """
        Handles Status data
        """
        self.statusdata = data
        self.numcurrent += min(data.drivers[0].current, data.drivers[1].current)
        self.totalcurrent += 1

    ### HEADING ###
    def createHeadingListener(self, topic):
        """
        Subscriber to Jackal Heading topic
        """
        self.headingnode = rospy.Subscriber(topic, MagneticField, self.updateHeading)

    def updateHeading(self, data):
        """
        Handles Heading data
        """
        self.headingdata = data
	
        # print(data)
        x = data.magnetic_field.x
        y = data.magnetic_field.y
        yaw = np.arctan2(y, x)

        # only use the calibration lists after calibration
        if self.has_calibrated:
            start_i = max(0, self.last_heading_i - 30)
            for i, v in enumerate(self.uncalib_i[start_i:]):
                if v < yaw:
                    continue
                yaw = self.calib_o[i+start_i]
                self.last_heading_i = i+start_i
                break

        if self.has_calibrated:
            declination_bias =  np.radians(270.0)
            yaw += declination_bias
            if yaw < 0:
                yaw += 2*np.pi
            if yaw > 2*np.pi:
                yaw -= 2*np.pi
        self.heading = yaw
        # print(f'heading: {self.heading}')

    def calibHeading(self):
        """
        Calibrates heading from magnetomer / compass sensor.
        Slowly rotates 360 degrees in place using magnetomer signal.
        Calculates ground truth mapping of mag signal to physical 
        heading using deg/sec from slow in-place circle.
        """

        print("starting yaw/heading/magnetomer calibration!")
        # wait for any filter time-lag efx to settle
        time.sleep(1)
        start_yaw = self.heading
        turn_speed = 0.2
        msg = self.makeTwist(0, 0, 0, 0, 0, turn_speed)
        # slowly turn until same heading/yaw
        time_elapsed = 0
        start_t = time.time()
        yaws = []
        while abs(start_yaw - self.heading) > 0.01 or time_elapsed < 5:
            self.drivernode.publish(msg)
            new_time_elapsed = time.time() - start_t
            # don't save more than ~10 messages a second
            if new_time_elapsed - time_elapsed < 0.1:
                continue
            time_elapsed = new_time_elapsed
            yaws.append([self.heading, time_elapsed])
        rad_per_sec = 2 * np.pi / time_elapsed

        # parallel arrarys
        self.uncalib_i = []
        self.calib_o = []
        for y in yaws:
            mapped_y = start_yaw + rad_per_sec * y[1]
            if mapped_y > np.pi:
                mapped_y -= 2 * np.pi
            self.uncalib_i.append(y[0])
            self.calib_o.append(mapped_y)

        print("number of calibration points: " + str(len(yaws)))
        # sort then in such a way to keep them parallel lists
        self.uncalib_i, self.calib_o = (list(l) for l in zip(*sorted(zip(self.uncalib_i, self.calib_o))))
        self.has_calibrated = True
        

    ### GPS ###
    def createGpsListener(self, topic):
        """
        Subscriber to Jackal GPS topic
        """
        self.gpsnode = rospy.Subscriber(topic, NavSatFix, self.updateGps)

    def updateGps(self, data):
        """
        Handles GPS data
        """

        if not math.isnan(data.latitude) and not math.isnan(data.longitude):
            self.gpsdata = data
            self.latitude = data.latitude
            self.longitude = data.longitude

    def createGPSaccListener(self, topic):
        """
        Subscriber to GPS accuracy topic
        """
        self.acc_node = rospy.Subscriber(topic, Float32, self.update_gps_acc)

    def update_gps_acc(self, m):
        self.gps_acc = m.data
        # print(f'gps_acc: {self.gps_acc}')

        self.numgpsacc += 1
        self.totalgpsacc += self.gps_acc

    ### ODOM ###
    def createOdomListener(self, topic):
        """
        Subscriber to Jackal ODOM topic
        """
        self.odomnode = rospy.Subscriber(topic, Odometry, self.updateOdom)

    def updateOdom(self, data):
        """
        Handles ODOM data
        """
        self.odomdata = data
        w = data.pose.pose.orientation.w
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z

        #ssert False
        _roll, _pitch, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])

        self.odomheading = yaw

    ### GPS-based turning ###
    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371e3  # Earth radius in meters

        d_lat = np.radians(lat2 - lat1)
        d_lon = np.radians(lon2 - lon1)

        a = np.sin(d_lat / 2) * np.sin(d_lat / 2) + \
	    np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * \
	    np.sin(d_lon / 2) * np.sin(d_lon / 2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

        return R * c

    def getAngleDistance(self, start, end):
        """
        Returns bearing from start coordinate to end coordinate
            start: (latitude, longitude) start GPS point
            end: (latitude, longitude) end GPS point
        """
        ang = np.arctan2((end[0] - start[0]), (end[1] - start[1]))
        distance = self.haversine(start[0], start[1], end[0], end[1])

        if ang < 0:
            ang = ang + 2*np.pi

        return ang, distance
    
    def turnToWaypoint(self, startlat, startlong, endlat, endlong):
        """
        Issues drive commands to turn Jackal to match the desired coordinates
            latitude, longitude: coordinates of destination
        """
        ang, dist = self.getAngleDistance((startlat, startlong),(endlat, endlong))

        while abs(self.heading - ang) > 0.1:
            if abs(self.heading - ang) < np.pi:
                if self.heading < ang:
                    self.Drive(0.0, 0.5)
                elif self.heading > ang:					
                    self.Drive(0.0, -0.5)
            else:
                if self.heading < ang:
                    self.Drive(0.0, -0.5)
                elif self.heading > ang:					
                    self.Drive(0.0, 0.5)
            self.rate.sleep()
            #print("HEADING " + str(self.heading) + " | BEARING " + str(ang))

        return dist

    ### startup & cleanup ###
    def awaitReadings(self):
        """
        Halts everything until there is successful data from each topic
        """
        print("Waiting for readings..")
        
        if self.lidar:
            print("Waiting for lidar...")
            while(self.lidardata is None):
                time.sleep(1)
            print("DONE")

        if self.status:
            print("Waiting for status...")
            while(self.statusdata is None):
                time.sleep(1)
            print("DONE")

        if self.heading:
            print("Waiting for heading...")
            while(self.headingdata is None):
                time.sleep(1)
            print("DONE")

        if self.gps:
            print("Waiting for GPS data...")
            while(self.gpsdata is None):
                time.sleep(1)
            print("DONE")

    def unregisterAll(self):
        if self.lidar:
            self.lidarnode.unregister
        if self.status:
            self.statusnode.unregister
        if self.heading:
            self.headingnode.unregister
        if self.gps:
            self.gpsnode.unregister


if __name__ == "__main__":

    # kill any previously running instances
    os.system("kill -9 $(pgrep -f 'python3 wifi_ros.py')")
    os.system("python3 wifi_ros.py &") # wifi pub
    os.system("kill -9 $(pgrep -f 'python3 grab_gps')")
    os.system("python3 grab_gps.py >/dev/null 2>&1 &") # phone gps pub
    os.system("kill -9 $(pgrep -f 'standalone image_proc/resize image:=/camera/image_color')")
    resize_cmd = "rosrun nodelet nodelet standalone image_proc/resize \
                  image:=/camera/image_color camera_info:=/camera/camera_info \
                  _scale_width:=0.5 _scale_height:=0.5 &"
    os.system(resize_cmd) # resize camera img 
    os.system("pkill -f 'throttle messages /image_proc'")
    os.system("rosrun topic_tools throttle messages /image_proc_resize/image 5 /throttled_img &")

    # Initialize Jackal Controller and Calibrate
    jackal = JackalController(headingtopic='gx5/mag', gpstopic='novatel/fix')
    jackal.awaitReadings()
    # kill ANY currently running rosbags!
    if args.rosbag:
        jackal.rosbag(False)
    if os.path.exists("calib.pkl"):
        with open("calib.pkl", "rb") as f:
            a = pickle.load(f, encoding='bytes')
            jackal.uncalib_i = a[0]
            jackal.calib_o = a[1]
            jackal.has_calibrated = True
    else:
        jackal.calibHeading()
        with open("calib.pkl", "wb") as f:
            pickle.dump([jackal.uncalib_i, jackal.calib_o], f)

    # Find place network if exists, otherwise create it
    network = PlaceNetwork()
    if os.path.exists("chkpt.pkl"):
        data = loadNetwork("chkpt")
        network.loadFromFile(data)
    else:
        print("chkpt not loaded")
        network.initAldritch(numcosts=6)
        network.initConnections()


    if args.rosbag:
        jackal.rosbag() # start recording

    if args.type == 'single':
        x_start = int(input("Enter starting x: "))
        y_start = int(input("Enter starting y: "))
        x_end = int(input("Enter ending x: "))
        y_end = int(input("Enter ending y: "))

        costmap = int(input("Enter costmap: "))

        wp_end = np.array([x_end, y_end])
        wp_start = np.array([x_start, y_start])

        p = network.spikeWave(wp_start, wp_end, costmap=[costmap])

        wpts = [network.cells[i].origin for i in p]
        pts = [network.points[i] for i in p]
        print("Path: ")
        print(pts[::-1])

        costs, reached = jackal.drivePath(wpts[::-1], network, pts[::-1])

        
        if len(p) != reached + 1:
            print("Full path not reached")
            p = p[len(p) - 2 - reached:]
            pts = pts[len(pts) - 1 - reached:]
            print("Reached up to")
            print(pts[::-1])

        with open("single_log.txt", "a") as file:
            file.write(str(pts[::-1])+'\n')
            file.write(str(costs)+'\n')

    elif args.type == 'spikewave':
        
        wp_end = np.array([args.x, args.y])
        wp_start = np.array([args.x, args.y])

        n1 = network.mapsizelat
        n2 = network.mapsizelon

        trials = args.trials
        network.first = True

        t = 0
        with open("log.txt", "a") as file:
            while t < trials:
                lf = levy_flight(2, 3)  # levy_flight will return a new waypoint
                # waypoint coordinates must be whole numbers
                new_wp = (wp_start[0] + round(lf[0]), wp_start[1] + round(lf[1]))
                while new_wp not in network.points:
                    lf = levy_flight(2, 3)
                    new_wp = (wp_start[0] + round(lf[0]), wp_start[1] + round(lf[1]))
                wp_end = np.copy(new_wp)

                print("Path %d from %d,%d to %d,%d" % (t+1, wp_start[0], wp_start[1], wp_end[0], wp_end[1]))

                if get_distance(wp_start, wp_end) > 1:
                    t += 1
                    p = network.spikeWave(wp_start, wp_end, costmap=[0, 1, 4, 5])

                    wpts = [network.cells[i].origin for i in p]
                    pts = [network.points[i] for i in p]
                    print("Path: ")
                    print(pts[::-1])

                    costs, reached = jackal.drivePath(wpts[::-1], network, pts[::-1])

                    if len(p) != reached + 1:
                        print("Full path not reached")
                        p = p[len(p) - 2 - reached:]
                        pts = pts[len(pts) - 1 - reached:]
                        print("Reached up to")
                        print(pts[::-1])
                    wp_start = np.array([pts[0][0], pts[0][1]])
                    # set wp_end to the end of the path just in case path was not reached.

                    #write path
                    file.write(str(pts[::-1])+'\n')

                    #UPDATE
                    network.eProp(costs, p)
                    saveNetwork(network, "wp_" + str(args.start+t))

    elif args.type == 'path':
        x_start = int(input("Enter starting x: "))
        y_start = int(input("Enter starting y: "))
        x_end = int(input("Enter ending x: "))
        y_end = int(input("Enter ending y: "))

        pathtype = str(input("spike or RRT"))

        wp_end = np.array([x_end, y_end])
        wp_start = np.array([x_start, y_start])

        if pathtype == 'spike':
            p = network.spikeWave(wp_start, wp_end, costmap=[0, 1, 4, 5])
        elif pathtype == 'RRT':
            p = network.RRTstar(wp_start, wp_end, costmap=[0, 1, 4, 5])
        elif pathtype == 'astar':
            p = network.astar(wp_start, wp_end, costmap=[0, 1, 4, 5])

        wpts = [network.cells[i].origin for i in p]
        pts = [network.points[i] for i in p]
        print("Path: ")
        print(pts[::-1])

        costs, reached = jackal.drivePathTest(wpts[::-1], pts[::-1])

        #Save to file
        with open(f"{pathtype}_{x_start}-{y_start}_{x_end}-{y_end}.txt", "a") as file:
            file.write(str(pts[::-1])+'\n')
            file.write(str(costs)+'\n')


    # elif args.type == 'goals':
    #     wp_end = np.array([10, 10])
    #     wp_start = np.array([10,10])

    #     n1 = network.mapsizelat
    #     n2 = network.mapsizelon

    #     trials = args.trials
    #     network.first = True

    #     waypoints = []

    #     if os.path.exists(args.list):
    #         with open(args.list, 'r') as f:
    #             for line in f:
    #                 line = line.strip()
    #                 parts = line.split(' ')
    #                 waypoint = (int(float(parts[0])), int(float(parts[1])))
    #                 waypoints.append(waypoint)
    #     else:
    #         print("File does not exist")
    #         exit()

    #     if args.trials != 0:
    #         wp_start = waypoints[args.start - 1]
    #         #FiX
    #         #wp_start = np.array([8, 15])


    #     for i, wps in enumerate(waypoints[args.start:]):
    #         wp_end = np.copy(wps)
    #         p = network.spikeWave(wp_start, wp_end, costmap=[0, 5])


    #         wpts = [network.cells[i].origin for i in p]
    #         pts = [network.points[i] for i in p]
    #         print("Path: ")
    #         print(pts[::-1])

    #         costs, reached = jackal.drivePath(wpts[::-1], network, pts[::-1])

    #         if len(p) != reached + 1:
    #             print("Full path not reached")
    #             p = p[len(p) - 2 - reached:]
    #             pts = pts[len(pts) - 1 - reached:]
    #             print("Reached up to")
    #             print(pts[::-1])
    #         wp_start = np.array([pts[0][0], pts[0][1]])
    #         # set wp_end to the end of the path just in case path was not reached.

    #         #UPDATE
    #         network.eProp(costs, p)
    #         saveNetwork(network, "wp_" + str(args.start+i))
    #         if i % 5:
    #             saveNetwork(network, "chkpt")
    #     saveNetwork(network, "chkpt")

        

            
        
    try:
        input("Press ENTER to terminate script")
    except KeyboardInterrupt: # hmm, doesn't work
        pass
    if args.rosbag:
        jackal.rosbag(False) # stop recording
    
    saveNetwork(network, "chkpt")

