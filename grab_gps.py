# gps from android hotspot
import rospy
from math import nan
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from http.server import BaseHTTPRequestHandler, HTTPServer

lat, lon, acc = nan, nan, nan

class S(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
    def do_GET(self):
        self._set_response()
        self.wfile.write("GET request for {}".format(self.path).encode('utf-8'))
        # useful for testing w/o GPS using CURL || wget
        global lat
        global lon
        global acc
        lat, lon, acc = 666, 666, 0 # test values
        #print("TEST SUCCESSFUL")
    def do_POST(self):
        global lat
        global lon
        global acc
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        fone_data = post_data.decode("utf-8").split("&")
        lat, lon, acc = fone_data
        lat, lon, acc = float(lat), float(lon), float(acc)
        self._set_response()

if __name__ == '__main__':
    # there's a threshold that below requires sudo
    port = 8080
    server_address = ('', port)
    httpd = HTTPServer(server_address, S)

    rospy.init_node("fone_gps")
    rate = rospy.Rate(4) # 4Hz
    lat_long_pub = rospy.Publisher("/fone_gps/fix", NavSatFix, queue_size=10)
    acc_pub = rospy.Publisher("/fone_gps/acc", Float32, queue_size=10)

    try:
        while not rospy.is_shutdown():
            httpd.handle_request()
            if lat != nan:
                msg = NavSatFix()
                msg.latitude = lat
                msg.longitude = lon
                lat_long_pub.publish(msg)
            if acc != nan:    
                msg = Float32()
                msg.data = acc
                acc_pub.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        pass    
    httpd.server_close()
