'''
Visualize gps signal variance from a rosbag
'''
import math
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
  nan_count = 0
  bag = rosbag.Bag(args.bag_file)
  for topic, msg, t in bag.read_messages(topics='/navsat/fix'):
    curr_lat = msg.latitude
    curr_lon = msg.longitude
    if prev_lat == None:
      print("only expecting this msg once @ beginning of bag")
    else:
      diff_lat = abs(curr_lat - prev_lat)
      diff_lon = abs(curr_lon - prev_lon)
      # scary triangle maths
      hypo = (diff_lat ** 2 + diff_lon ** 2) ** 0.5
      if math.isnan(hypo):
        nan_count += 1
      else:
        diff_poses.append(hypo)
    prev_lat = curr_lat
    prev_lon = curr_lon
  bag.close()
  print("number of NaNs: ", nan_count)
  plt.hist(diff_poses, bins=10)
