import os
import tf
import time
import math
import rospy
import pickle
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from placecell import PlaceNetwork
from jackal_msgs.msg import Feedback
from geometry_msgs.msg import Twist, PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import LaserScan, Imu, MagneticField, NavSatFix


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
                 headingtopic = '/imu/data',
                 gps = True,
                 gpstopic = 'navsat/fix',
                 odom = True,
                 odomtopic = 'odometry/filtered',
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

    def rosbag(self, record=True):
        # Records rosbag for offline analysis. 
        
        # start recording
        if record:
            self.bag_file = f'{int(time.time())}' + '.bag'
            # @TODO don't hard-code the wifi topic
            bash_cmd = f'rosbag record -O {self.bag_file} {self.lidartopic} {self.statustopic} {self.headingtopic} '
            bash_cmd += f'{self.gpstopic} {self.odomtopic} {self.gps_acc_topic} /wifi_strength &' # background
            os.system(bash_cmd)
        else:
            # try stopping any already running rosbag recordings
            os.system(f'kill $(pgrep -f "rosbag record -O {self.bag_file}")')

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


    def driveToWaypoint(self, latitude, longitude):
        """
        Drives Jackal to the coordinates. Returns a tuple (bool, float). bool is true for successfully reaching false for unsuccessful, float is the cost.
        latitude, longitude: coordinates
        """
        #self.turnToWaypoint(latitude, longitude)

		#Start tracking cost after turning to directionb
        ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))
        while dist > 0.75:
            self.driveToward(ang, dist)
            print("Distance to waypoint " + str(dist) + " | Heading " + str(self.heading) + " Angle to bearing: " + str(ang))
            ang, dist = self.getAngleDistance((self.latitude, self.longitude), (latitude, longitude))

    def driveToward(self, angle, dist):
        """
        Issues a single command toward the desired angle, returns distance 
        """
        padding = np.pi/12
        #if self.blocked:
            #self.Drive(self.drivespeed, self.blockdir * self.maxturnspeed)
        #    self.avoidancetime += 1
        #else:
        if self.heading < angle - padding:
            self.Drive(0.0, 0.5)
        elif self.heading > angle  + padding:					
            self.Drive(0.0, -0.5)
        else:
            self.Drive(0.5, 0)

        #self.drivetime += 1
        self.rate.sleep()

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

        while abs(self.heading - ang) > 0.05:
            self.Drive(0, 0.25)
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
    os.system("kill -9 $(pgrep -f 'python wifi_ros.py')")
    os.system("python wifi_ros.py &")
    os.system("kill -9 $(pgrep -f 'python grab_gps')")
    os.system("python grab_gps.py &")

    test = JackalController(headingtopic='gx5/mag', gpstopic='fone_gps/fix')
    test.awaitReadings()
    if os.path.exists("calib.pkl"):
        with open("calib.pkl", "rb") as f:
            a = pickle.load(f, encoding='bytes')
            test.uncalib_i = a[0]
            test.calib_o = a[1]
            test.has_calibrated = True
    else:
        test.calibHeading()
        with open("calib.pkl", "wb") as f:
            pickle.dump([test.uncalib_i, test.calib_o], f)

    test.rosbag() # start recording

    # SBSG waypoints
    # [33.64753, -117.83918] # Lab
    # [33.64755, -117.83737] # Parking
    # [33.65073, -117.83830] # Two asprin
    # [33.64542, -117.84073] # Einsteins

    # Park waypoints
    # [33.64659, -117.84304]   # Intersection
    # [33.64721, -117.84289]   # End of dirt road
    # [33.646727, -117.843020] # Halfway dirt road

    '''
    network = PlaceNetwork()
    network.initAldritch()
    network.initConnections()

    path = network.spikeWave(network.points[(0, 0)], network.points[(0, 0)])
    for point in path[::-1]:
        print("Moving to waypoint: " + str(network.cells[point].origin[0]) + ", "  + str(network.cells[point].origin[1]))
        test.driveToWaypoint(network.cells[point].origin[0], network.cells[point].origin[1])
        test.turnToWaypoint(test.latitude, test.longitude, network.cells[point].origin[0], network.cells[point].origin[1])
    '''

    try:
        input("Press ENTER to terminate script")
    except KeyboardInterrupt: # hmm, doesn't work
        pass
    test.rosbag(False) # stop recording
