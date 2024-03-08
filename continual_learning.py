import sys
from utils import get_distance
from queue import PriorityQueue
import os
from dataclasses import dataclass, field
from typing import Any, Tuple
import matplotlib.pyplot as plt
from placecell import PlaceNetwork, loadNetwork
from tqdm import tqdm
import numpy as np
import pickle
import matplotlib.pyplot as plt
import random

def calc_total_cost(network, path, costmap):
    r_path = path[::-1]
    wgts = network.normalizeWeights(costmap)

    total = 0
    for i in range(len(r_path) - 1):
        total += wgts[(r_path[i], r_path[i + 1])]
    return total

if os.exists("avgs.pkl"):
    avgs = pickle.load(open("avgs.pkl", "rb"))
    plt.figure()
    plt.plot(avgs)
    plt.show()
    sys.exit()

true_network = PlaceNetwork()
true_data = loadNetwork("fixed_wgts")
true_network.loadFromFile(true_data)

wps = []
st_ends = []

for cell in true_network.cells:
    wps.append((true_network.points[cell.ID][0], true_network.points[cell.ID][1]))

print("Generating start-end pairs")
for start in wps:
    for end in wps:
        if start == end or get_distance(start, end) < 2:
            continue
        st_ends.append((start, end))

avgs = []

for i in range(351):
    if os.path.exists("wps/wp_{}.pkl".format(i)):
        print("Loading wp_{}".format(i))
        test_network = PlaceNetwork()
        test_data = loadNetwork("wps/wp_{}".format(i))
        test_network.loadFromFile(test_data)

        total_cost = 0

        tests = random.sample(st_ends, 1000)
        for pt in tqdm(st_ends):
            p = test_network.spikeWave(pt[0], pt[1], costmap=[0, 1, 4, 5])
            total_cost += calc_total_cost(true_network, p, [0, 1, 4, 5])

        avg_cost = total_cost / len(st_ends)
        avgs.append(avg_cost)

plt.figure()
plt.plot(avgs)
plt.show()

pickle.dump(avgs, open("avgs.pkl", "wb"))


