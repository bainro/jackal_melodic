'''
Visualize gps signal variance from a rosbag
'''
import rosbag
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
_help = "path to previously recorded rosbag"
parser.add_argument("--bag_file", type=str, required=True, help=_help)
args = parser.parse_args()

if __name__ == "__main__":
  prev_lat, prev_lon = None, None
  diff_poses = []
  bag = rosbag.Bag(args.bag_file)
  for topic, msg, t in bag.read_messages(topics='/navsat/fix'):
    print('hmm')
    curr_lat = msg.latitude
    curr_lon = msg.longitude
    if prev_lat == None:
      print("only expecting this msg once @ beginning of bag")
    else:
      diff_lats = abs(curr_lat - prev_lat)
      diff_lons = abs(curr_lon - prev_lon)
      # scary triangle maths
      diff_poses.append((diff_lat ** 2 + diff_lon ** 2) ** 0.5)
    prev_lat = curr_lat
    prev_lon = curr_long
  bag.close()
  plt.hist(diff_poses, bins=10)
