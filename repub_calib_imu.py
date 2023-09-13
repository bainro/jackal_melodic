# apply calibration results to imu's magnetometer & publish in ROS
### realized midway thru we no need...
import os
import sys
import rospy
from sensor_msgs.msg import MagneticField

mag_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
def repub_mag(msg):
  x = msg.magnetic_field.x
  y = msg.magnetic_field.y
  z = msg.magnetic_field.z
  w = msg.magnetic_field.w
  mag_pub.publish(msg)

if __name__ == '__main__':
    assert len(sys.argv) == 2, "pass in name of the mag topic" 
    mag_topic = str(sys.argv[-1])
    .lidarnode = rospy.Subscriber(mag_topic, MagneticField, repub_mag)
    rospy.init_node("calib_mag")
    rate = rospy.Rate(15) # Hz
    pub = rospy.Publisher("/wifi_strength", Int32, queue_size=10)
  
    while not rospy.is_shutdown():
        msg = Int32()
        msg.data = wifi_sig
        pub.publish(msg)
        rate.sleep()
