# get roll, pitch, and yaw from imu sensor
import tf2
import sys
import rospy
from sensor_msgs.msg import Imu

def get_rpy(m):
  x = m.orientation.x
  y = m.orientation.y
  z = m.orientation.z
  w = m.orientation.w
  roll, pitch, _yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
  print(f'roll: {roll}')
  print(f'pitch: {pitch}')

if __name__ == '__main__':
    assert len(sys.argv) == 2, "pass in name of the mag topic" 
    rospy.init_node("calib_mag")
    imu_topic = str(sys.argv[-1])
    lidarnode = rospy.Subscriber(imu_topic, Imu, get_rpy)
    
    rate = rospy.Rate(4) # Hz
  
    while not rospy.is_shutdown():
        rate.sleep()
