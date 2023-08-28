'''
Visualize gps and imu data from a rosbag
'''
import os
import cv2
import time
import math
import rosbag
import argparse
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from pynput import keyboard as kb
from tf import transformations as t

parser = argparse.ArgumentParser()
_help = "use this image as the background (eg satelite.png)"
parser.add_argument("--map_file", type=str, default=False, help=_help)
_help = "path to previously recorded rosbag"
parser.add_argument("--bag_file", type=str, required=True, help=_help)
_help = "directory to save the training data"
parser.add_argument("--out_dir", type=str, required=True, help=_help)
_help = "rotation of path in radians"
parser.add_argument('--rot', type=float, default=0.006989, help=_help)
_help = "scale of path"
parser.add_argument('--scale', type=float, default=20.0, help=_help)
_help = "higher moves the path more to the right"
parser.add_argument('--x_off', type=float, default=24.38, help=_help)
_help = "higher moves the path further down"
parser.add_argument('--y_off', type=float, default=10.5, help=_help)
_help = "size in pixels to save the dataset images"
parser.add_argument('--size', type=int, default=256, help=_help)
args = parser.parse_args()

if __name__ == "__main__":
  lats, longs, headings, gps_accs = [], [], [], []
  bag = rosbag.Bag(args.bag_file)
  # helps keep the 2 parallel
  new_acc = True
  for topic, msg, t in bag.read_messages():
    if "fone_gps/fix" in topic:
      if new_acc:
        lats.append(msg.latitude)
        longs.append(msg.longitude)
        new_acc = False      
    elif "fone_gps/acc" in topic: # "gx5/mag":
      if new_acc: 
        continue
      new_acc = True
      #x = msg.magnetic_field.x
      #y = msg.magnetic_field.y
      #yaw = np.arctan2(y, x)
      #headings.append(yaw)
      gps_accs.append(msg.data)
    else:
      continue
  bag.close()

  print(f'Number of datapoints: {len(lats)}')
  if not (len(lats) == len(longs) == len(gps_accs)):
    # headings = headings[:-1]
    lats = lats[:-1]
    longs = longs[:-1]
  _str = f'Not parallel lists! ({len(lats)},{len(longs)},{len(gps_accs)})'
  assert len(lats) == len(longs) == len(gps_accs),  _str

  with open(os.path.join(args.out_dir, "meta_data.csv"), "w") as meta_data_file:
    meta_data_file.write("gps_acc,Longitude,Latitude\n")
    for i in range(len(lats)):
      meta_data_file.write(f'{gps_accs[i]:.1f},{longs[i]},{lats[i]}\n')

  # normalize lat & long
  _lats, _longs = [], []
  for l1, l2 in zip(lats, longs):
    # x is reverse for some reason (probably because of the neg)
    _lats.append(1 - ((l1 - min(lats)) / (max(lats) - min(lats))))
    _longs.append((l2 - min(longs)) / (max(longs) - min(longs)))
  lats, longs = _lats, _longs
  _str = f'min,max lats: ({min(lats)},{max(lats)})'
  assert min(lats) >= 0 and max(lats) <= 1.0, _str
  _str = f'min,max longs: ({min(longs)},{max(longs)})'
  assert min(longs) >= 0 and max(longs) <= 1.0, _str
  
  # use keys to translate, rotate, & scale the path
  rot = args.rot
  scale = args.scale
  x_off = args.x_off
  y_off = args.y_off
  
  # key callback generator
  def key_cb_gen(dv, dSoR):
    def ky_cb(shift, scale_or_rot, offset):      
      if shift:
        scale_or_rot = scale_or_rot + dSoR
      else:
        # translate points
        offset = offset + dv
      return scale_or_rot, offset
    return ky_cb
  
  up_cb = key_cb_gen(-0.15, 0.1)
  down_cb = key_cb_gen(0.15, -0.1)
  left_cb = key_cb_gen(-0.15, 0.003)
  right_cb = key_cb_gen(0.15, -0.003)        
  
  shift_on = False
  enter_pressed = False
  
  def kr(key): # key released
    global shift_on, enter_pressed
    global rot, scale, x_off, y_off
    if key == kb.Key.shift:
      shift_on = False
    elif key == kb.Key.left:
      rot, x_off = left_cb(shift_on, rot, x_off)
    elif key == kb.Key.right:
      rot, x_off = right_cb(shift_on, rot, x_off)
    elif key == kb.Key.down:
      scale, y_off = down_cb(shift_on, scale, y_off)
    elif key == kb.Key.up:
      scale, y_off = up_cb(shift_on, scale, y_off)
    elif key == kb.Key.enter:
      enter_pressed = True
    return False
  
  def kp(key): # key pressed
    if key == kb.Key.shift:
      global shift_on
      shift_on = True
   
  # load the picture of the map
  map_img = None
  map_file = args.map_file or "/tmp/map.png"
  with open(map_file, 'rb') as mf:
    map_img = plt.imread(mf)
  og_map_shape = map_img.shape
  
  colors = cm.gist_rainbow(np.linspace(0, 0.85, len(lats)))
  fig = plt.figure(figsize=(36,12))
  while not enter_pressed:
    plt.clf()
    plt.title("use arrow keys and shift to align the path")
    plt.imshow(map_img, 
               resample=False, 
               interpolation='none', 
               cmap='gray', 
               vmin=0, 
               vmax=255)
    trans_path_x, trans_path_y = [], []
    for i in range(len(lats)):
      x = lats[i] * math.cos(rot) - longs[i] * math.sin(rot)
      trans_path_x.append(scale * (x + x_off))
      y = longs[i] * math.cos(rot) + lats[i] * math.sin(rot)
      trans_path_y.append(scale * (-y + y_off))
    # overlay the path on the map 
    plt.scatter(x=trans_path_x, y=trans_path_y, c=colors, s=3)
    plt.show(block=False)
    plt.pause(0.001)
    with kb.Listener(on_press=kp, on_release=kr) as listener:
      listener.join() 
  
  print("\n\nrot: ", rot), print("scale: ", scale)
  print("x_off: ", x_off), print("y_off: ", y_off)
  
  plt.scatter(x=trans_path_x[0], 
              y=trans_path_y[0], 
              c=colors[0], 
              label="start",
              s=25)
  plt.scatter(x=trans_path_x[-1], 
              y=trans_path_y[-1], 
              c=colors[-1],
              label="end",
              s=25)
  l = plt.legend(loc="lower right", fontsize=15)
  # hack to scale legend's icons with bigger font size
  l.legendHandles[0]._sizes = [200]
  l.legendHandles[1]._sizes = [200]
  fig.savefig('/tmp/overlay.svg', format='svg', dpi=1200)
  plt.clf()

  print("DONE!")
