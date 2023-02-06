# Debugging script to see how much GPS signal bounced around
import csv
from matplotlib import pyplot as plt

# distance between points
dx_between_pts = []

prev_lat, prev_long = 0, 0
with open('2_2_23_gps.csv', mode ='r') as f:
  csv_f = csv.reader(f)
  for line in csv_f:
    lat, long = line[1:3]
    print(lat,long)
    if prev_lat != 0:
      dlat = lat - prev_lat
      dlong = long - prev_long
      dx = (dlat**2 + dlong**2) ** .5
      dx_between_pts.append(dx)
    prev_lat = lat
    prev_long = long
          
# Creating histogram
fig, ax = plt.subplots(figsize =(10, 7))
ax.hist(dx_between_pts, bins = [0, 25, 50, 75, 100])

print("max: ", max(dx_between_pts), " min: ", min(dx_between_pts))

# Show plot
plt.show()
