import os
import tf
import time
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from jackal_msgs.msg import Feedback
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, LaserScan, Imu, MagneticField

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
                 odomtopic = 'odometry/filtered'):
        
        # Creates publisher for making drive commands
        self.createSimpleDriver() 
        # Creates publisher for making goal commands
        self.createGoalPublisher() 

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
        self.has_calibrated = False

        if lidar:
            self.createLidarListener(lidartopic)
        if status:
            self.createStatusListener(statustopic)
        if heading:
            self.createHeadingListener(headingtopic)
        if gps:
            self.createGpsListner(gpstopic)
        if odom:
            self.createOdomListener(odomtopic)

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
        
        msg.pose.orientation.x = self.odomdata.pose.pose.orientation.x
        msg.pose.orientation.y = self.odomdata.pose.pose.orientation.y
        msg.pose.orientation.z = self.odomdata.pose.pose.orientation.z
        msg.pose.orientation.w = self.odomdata.pose.pose.orientation.w
        self.goalnode.publish(msg)

    def moveToGoal(self, startlat, startlong, endlat, endlong):
        d = self.turnToWaypoint(startlat, startlong, endlat, endlong)

        # Wait a bit for the heading to stabilize
        time.sleep(3)
        x = self.odomdata.pose.pose.position.x + d*np.cos(self.odomheading)
        y = self.odomdata.pose.pose.position.y + d*np.sin(self.odomheading)

        self.publishGoal(x, y)

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
        declination_bias = 0 # not true, but good enough for dbg
        x = data.magnetic_field.x
        y = data.magnetic_field.y
        yaw = np.arctan2(y, x) + declination_bias

        # only use the calibration lists after calibration
        if self.has_calibrated:
            # can speed this search up by retaining the last found index and searching 
            # relative to that next time we update the heading
            for i, v in enumerate(self.uncalib_i):
                if v < yaw:
                    continue
                yaw = self.calib_o[i]
                break

        self.heading = yaw
        print('self.heading: ' + str(self.heading))

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
    def createGpsListner(self, topic):
        """
        Subscriber to Jackal GPS topic
        """
        self.gpsnode = rospy.Subscriber(topic, NavSatFix, self.updateGps)

    def updateGps(self, data):
        """
        Handles GPS data
        """
        self.gpsdata = data

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
        ang = np.arctan2((end[1] - start[1]), (end[0] - start[0]))
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
            while(self.gps is None):
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

    test = JackalController(headingtopic='gx5/mag')
    test.awaitReadings()
    test.calibHeading()

    #start = [test.gpsdata.latitude, test.gpsdata.longitude]
    #end1 = [33.64659, -117.84304] #HERE
    #end2 = [33.64721, -117.84289] #FAR
    #end3 = [33.64542, -117.84073] #EINSTEIN
    
    #INDOOR
    #end1 = [33.64753, -117.83918]
    #end2 = [33.65073, -117.83830]

    #test.moveToGoal(start[0], start[1], end1[0], end1[1])
    #end2 = [33.64755, -117.83737]
    #end3 = [33.64542, -117.84072]
    #test.turnToWaypoint(end2[0], end2[1], end1[0], end1[1])

    raw_input("Press ENTER to terminate script")
