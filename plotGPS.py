# Debugging script to see how much GPS signal bounced around
import csv
import math
import numpy as np
from matplotlib import pyplot as plt

# distance between points
dx_between_pts = []

prev_lat, prev_long = 0, 0
with open('2_2_23_gps.csv', mode ='r') as f:
  csv_f = csv.reader(f)
  for i, line in enumerate(csv_f):
    # column/header line (e.g. "time, lat, long, alt")
    if i == 0:
      continue
    lat, long = float(line[1]), float(line[2])
    if prev_lat != 0:
      dlat = lat - prev_lat
      dlong = long - prev_long
      dx = (dlat**2 + dlong**2) ** .5
      if not math.isnan(dx):
        dx_between_pts.append(dx)
    prev_lat = lat
    prev_long = long
          
dx_min = min(dx_between_pts)
dx_max = max(dx_between_pts)
print("max: ", dx_max, " min: ", dx_min) 
dx_sorted = np.sort(dx_between_pts)
plt.plot(dx_sorted)
assert dx_sorted[0] == dx_min, f"{dx_sorted[0]} != {dx_min}"
assert dx_sorted[-1] == dx_max, f"{dx_sorted[-1]} != {dx_max}"
plt.show()

#################
### Phone GPS ###
#################
import gpxpy
import gpxpy.gpx

gpx_file = open('phone_gps.gpx', 'r')
gpx = gpxpy.parse(gpx_file)

for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
print(len(gpx.tracks))

'''
# Creating histogram
fig, ax = plt.subplots(figsize = (10, 7))
min_dx = min(dx_between_pts)
max_dx = max(dx_between_pts)
bin_width = (max_dx - min_dx) / 5
ax.hist(dx_between_pts, bins = np.arange(min_dx, max_dx, bin_width))
# Show plot
plt.show()
'''
