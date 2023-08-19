import os
import tf
import math
import rospy
from sensor_msgs.msg import Imu
from HIMUServer import HIMUServer

class PhoneListener:
    def __init__(self, serverInstance):
        self.__server = serverInstance
        self.make_imu_pubber()
        self.r = 0 # roll
        self.p = 0 # pitch
        self.y = 0 # yaw

    def notify (self, sensor_data):
        # non-empty case
        if len(sensor_data) > 0:
            r, p, y = sensor_data[0][0]
            print(r)
            r, p, y = r.encode("utf-8"), p.encode("utf-8"), y.encode("utf-8")
            self.r, self.p, self.y = float(r), float(p), float(y)
            self.pub_imu()

    def make_imu_pubber(self):
        self.imu_pubber = rospy.Publisher('hyperIMU/data', Imu, queue_size=10)

    def pub_imu(self):
        msg = Imu()
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()

        q = tf.transformations.quaternion_from_euler(self.r, self.p, self.y)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        
        self.imu_pubber.publish(msg)
        rospy.Rate(10) #10Hz

if __name__ == "__main__": 

    rospy.init_node('hyperIMU')
    hIMU_server = HIMUServer()
    #Creating listener and adding it to the server instance:
    orientation_listener = PhoneListener(hIMU_server)
    hIMU_server.addListener(orientation_listener)

    print("\n\nASSUMING 1 SENSOR (orientation) FROM ANDROID PHONE FOR NOW\n\n")
    print("Connect the hyperIMU andriod app over hostspot now!")

    hIMU_server.timeout = 20 # seconds
    hIMU_server.start("TCP", 2055) # port 2055

    
