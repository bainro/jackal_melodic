'''
Coverts a rosbag into a dataset for DCNNs to estimate 2D location
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
  # run a bag in offline localization-only mode 
  _ = os.system("rosparam set use_sim_time true")
  # get just the pose position (x,y) and the corresponding timestamp (secs)
  os.system("grep -C4 position /tmp/traj.txt | grep -e 'x:' > /tmp/x.log")
  os.system("grep -C4 position /tmp/traj.txt | grep -e 'y:' > /tmp/y.log")
  os.system("grep -C4 position /tmp/traj.txt | grep -e 'secs:' | grep -v 'nsecs' > /tmp/secs.log")
  os.system("grep -C4 position /tmp/traj.txt | grep -e 'nsecs:' > /tmp/nsecs.log")
  os.system("grep -C4 'w:' /tmp/traj.txt | grep -e 'z:' > /tmp/rot_z.log")
  os.system("grep     'w:' /tmp/traj.txt > /tmp/rot_w.log")
  
  def extract(fname, str_bias):
    path_var = []
    with open(fname, 'r') as f:
      for l in f.readlines():
        l = l.strip()
        path_var.append(float(l[str_bias:]))
    return path_var
  
  # read the files into parallel lists
  path_x = extract('/tmp/x.log', 3)
  path_y = extract('/tmp/y.log', 3)
  # Quarternion rotation. We only need z & w
  path_z = extract('/tmp/rot_z.log', 3)  
  path_w = extract('/tmp/rot_w.log', 3)
  path_yaw = [] # euler rotation from z & w
  path_secs = extract('/tmp/secs.log', 6)
  path_nsecs = extract('/tmp/nsecs.log', 6)
  for i in range(len(path_secs)):
    path_secs[i] = path_secs[i] + path_nsecs[i] / 1e9
  del path_nsecs # don't need nsecs anymore
  
  # change angle in radians to filter consecutive poses
  filter_dr = args.path_filter_r
  filter_dx = args.path_filter_x
  # last non-filtered (i.e. included) pose. 
  last_pt = [math.inf, math.inf, math.inf] 
  del_count = 0
  # filter out poses based on (dx, dr) wrt last included pose
  for i in range(len(path_secs)):
    i = i - del_count
    x = last_pt[0] - path_x[i]
    y = last_pt[1] - path_y[i]
    # pythagorean theorem
    dx = (x**2 + y**2) ** 0.5
  
    rpy = t.euler_from_quaternion([0, 0, path_z[i], path_w[i]])
    yaw = rpy[2] + math.pi # make smallest possible value == 0
    # have to check for wrap around!
    if abs(yaw - last_pt[2]) > math.pi:
      dr = 2 * math.pi - abs(yaw - last_pt[2]) 
    else:
      dr = abs(yaw - last_pt[2])
  
    if dx > filter_dx or dr > filter_dr:
      last_pt = [path_x[i], path_y[i], yaw]
      path_yaw.append(yaw)
    else: # bye-bye!
      del path_x[i], path_y[i], path_z[i], path_w[i], path_secs[i]
      del_count = del_count + 1
  
  print("Number of datapoints after filtering: ", len(path_x))
  assert len(path_secs) == len(path_x) == len(path_y), "No longer parallel lists!"
  assert len(path_y) == len(path_z) == len(path_w), "No longer parallel lists!"
  
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
  map_file = args.map_file or "/tmp/map.pgm"
  with open(map_file, 'rb') as mf:
    map_img = plt.imread(mf)
  og_map_shape = map_img.shape
  
  colors = cm.gist_rainbow(np.linspace(0, 0.85, len(path_x)))
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
    for i in range(len(path_x)):
      x = path_x[i] * math.cos(rot) - path_y[i] * math.sin(rot)
      trans_path_x.append(scale * (x + x_off))
      y = path_y[i] * math.cos(rot) + path_x[i] * math.sin(rot)
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
 
  def rotate_image(im, x, y, qz, qw):
    # print("assumes HxWxC image format!")
    rotation_pt = (int(x),int(y))
    _roll, _pitch, yaw = t.euler_from_quaternion([0, 0, qz, qw])
    yaw = yaw * -1 # flip rotation direction
    # offset to make the robot look up wrt to the map
    yaw = yaw + (math.pi / 2)
    yaw_degs = yaw * 180 / math.pi
    rot_mat = cv2.getRotationMatrix2D(rotation_pt, yaw_degs, 1.0)
    rot_img = cv2.warpAffine(im, rot_mat, im.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rot_img
  
  # global map perspective's width (centered at robot)
  gmp_w = args.gmp_w
  # add padding to simplify edge cases after rotating and remove later.
  if len(map_img.shape) == 3: # e.g. RGB
    # assumes padding color is same as top-left map px
    tl_color = map_img[0,0,:]
    old_shape = list(map_img.shape)
    padding_shape = [2*gmp_w, 2*gmp_w, 0]
    new_shape = [sum(x) for x in zip(old_shape, padding_shape)]
    _tmp_map = np.zeros(tuple(new_shape))
    _tmp_map[...] = tl_color
    _tmp_map[gmp_w:-gmp_w, gmp_w:-gmp_w, :] = map_img[...]
    map_img = _tmp_map
  else: # e.g. grayscale
    tl_color = 205 # map_img[0,0]
    old_shape = list(map_img.shape)
    padding_shape = [2*gmp_w, 2*gmp_w]
    new_shape = [sum(x) for x in zip(old_shape, padding_shape)]
    _tmp_map = np.zeros(tuple(new_shape))
    _tmp_map[...] = tl_color
    _tmp_map[gmp_w:-gmp_w, gmp_w:-gmp_w] = map_img[...]
    map_img = _tmp_map
      
  prior_data = 0
  if args.combine:
    with open(os.path.join(args.out_dir, "meta_data.csv"), "r") as meta_file:
      prior_data = len(meta_file.readlines())  
  else:
    _ = os.system(f"rm -fr {args.out_dir} > /dev/null 2>&1")
    os.makedirs(args.out_dir, exist_ok=True)
    
  for i in range(len(trans_path_x)):
    if len(map_img.shape) == 3: # e.g. RGB
      gmp_img = np.zeros(shape=(gmp_w, gmp_w, 3))
    else:
      gmp_img = np.zeros(shape=(gmp_w, gmp_w))
    # rotation origin
    rx = trans_path_x[i] + gmp_w
    ry = trans_path_y[i] + gmp_w
    rot_map = rotate_image(map_img, rx, ry, path_z[i], path_w[i])
    # crop out around the robot
    x_start = int(trans_path_x[i] + gmp_w / 2)
    x_end = int(trans_path_x[i] + 3 * gmp_w / 2)
    y_start = int(trans_path_y[i] + gmp_w / 2)
    y_end = int(trans_path_y[i] + 3 * gmp_w / 2)
      
    if len(rot_map.shape) == 3: # e.g. RGB
      gmp_img[:, :, :] = rot_map[y_start:y_end, x_start:x_end, :]
    else: # e.g. grayscale
      gmp_img[:, :] = rot_map[y_start:y_end, x_start:x_end]
    
    gmp_img = cv2.resize(gmp_img, dsize=(args.size, args.size), 
                         interpolation=cv2.INTER_AREA) 
    if len(gmp_img.shape) == 3: # eg RGB
      cv2.imwrite(os.path.join(args.out_dir, f'{i + prior_data}_map.png'), gmp_img*255)
    else:
      cv2.imwrite(os.path.join(args.out_dir, f'{i + prior_data}_map.png'), gmp_img)
    #if i == 0:
      #plt.title("verify map region of interest quality")
      #plt.imshow(gmp_img, cmap='gray', vmin=0, vmax=255)
      #plt.show()
    print(i)

  # save each FPV image with the corresponding GMP image
  bag = rosbag.Bag(args.bag_file)
  with open(os.path.join(args.out_dir, "meta_data.csv"), "a") as meta_data_file:
    if prior_data == 0:
      meta_data_file.write("frame,time,heading,x,y\n")
    i = 0
    # @TODO make CLI arg instead of hard-coded
    time_gap = 5
    cam_img = None
    for topic, msg, _t in bag.read_messages(topics=['/image_proc_resize/image']):
      if i >= len(path_x):
        break
      msg_t = msg.header.stamp.secs + (msg.header.stamp.nsecs / 1e9)
      if msg_t < path_secs[i]:
        continue
      assert_str = "assuming width is 2nd dimension"
      assert og_map_shape[1] > og_map_shape[0], assert_str
      norm_x = trans_path_x[i] / og_map_shape[1]
      norm_y = trans_path_y[i] / og_map_shape[0]
      assert_str = "normalizing should result in values between 0 & 1"
      assert norm_x >= 0 and norm_x <= 1, assert_str
      assert norm_y >= 0 and norm_y <= 1, assert_str
      assert msg.width > msg.height, "image width must be greater than image height"
      cam_img = np.asarray(list(msg.data), dtype=np.float32)
      cam_img = cam_img.reshape((msg.height, msg.width, 3))
      # whether still need to grab certain history channels
      ch1 = True
      # @TODO HORRIBLY INEFFICIENT XD 
      # @TODO Grab from future instead of past or rolling!
      for _, _msg, _t in bag.read_messages(topics=['/image_proc_resize/image']):
        _msg_t = _msg.header.stamp.secs + (_msg.header.stamp.nsecs / 1e9)
        # history channels
        if ch1 and _msg_t >= path_secs[i] - time_gap * 2:
          if _msg_t == msg_t:
            break
          # only do this once
          ch1 = False
          _cam_img = np.asarray(list(_msg.data), dtype=np.float32)
          cam_img[:,:,0] = _cam_img.reshape((_msg.height, _msg.width, 3))[:,:,0]
          continue
        if _msg_t >= path_secs[i] - time_gap:
          _cam_img = np.asarray(list(_msg.data), dtype=np.float32)
          cam_img[:,:,1] = _cam_img.reshape((_msg.height, _msg.width, 3))[:,:,0]
          break
      # not enough time since bag start to make history ch image!
      if ch1:
        i += 1
        continue
      meta_data_file.write("%s,%s,%.2f,%f,%f\n" % (i+prior_data, path_secs[i], path_yaw[i], norm_x, norm_y))    
      # crop to center
      x_offset = int((msg.width - msg.height) // 2)
      cam_img = cam_img[:, x_offset:-x_offset, :]
      assert_str = f"image should be square. {cam_img.shape[1]} != {cam_img.shape[0]}"
      assert cam_img.shape[0] == cam_img.shape[1], assert_str
      resize_dims = (args.size, args.size)
      fpv_img = cv2.resize(cam_img, dsize=resize_dims, interpolation=cv2.INTER_AREA)
      save_name = os.path.join(args.out_dir, f'{i + prior_data}_camera.png')
      cv2.imwrite(save_name, fpv_img[...,::-1])
      i += 1
      if i == 1:
        plt.title("verify first person view (i.e. camera image)")
        plt.imshow(fpv_img/255)
        plt.show()
  bag.close()
  print("DONE!")
