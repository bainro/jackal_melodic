# publish wifi's signal strength in ROS
import os
import rospy
from std_msgs.msg import Int32

if __name__ == '__main__':
    rospy.init_node("wifi_strength")
    rate = rospy.Rate(2) # ROS Rate at 2Hz
    pub = rospy.Publisher("/wifi_strength", Int32, queue_size=10)

    wifi_sig = 0
    wifi_ssid = "'UCInet Mobile Access'"
    nmcli_cmd = f'nmcli d wifi | grep {wifi_ssid}' # BASH

    while not rospy.is_shutdown():
        msg = Int32()
        nmcli = os.popen(nmcli_cmd).read()
        if len(nmcli) > 0:
            h_cmd = f'nmcli d wifi | grep BSSID | grep SIGNAL'
            header = os.popen(h_cmd).read()
            start_i = header.find('SIGNAL')
            wifi_sig = nmcli[start_i:start_i+4]
            wifi_sig = int(wifi_sig.strip())
        msg.data = wifi_sig
        pub.publish(msg)
        rate.sleep()