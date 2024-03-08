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

def wgt_mse(network, true_network, costmap):
    wgts = network.normalizeWeights(costmap)
    true_wgts = true_network.normalizeWeights(costmap)

    mse = 0
    for pred, true in zip(wgts.values(), true_wgts.values()):
        mse += (pred - true) ** 2
    return mse

true_network = PlaceNetwork()
true_data = loadNetwork("wps/wp_350")
true_network.loadFromFile(true_data)


wps = []
st_ends = []

totalloss = []
updates = []
for i in range(351):
    if os.path.exists("wps/wp_{}.pkl".format(i)):
        updates.append(i)
        print("Loading wp_{}".format(i))
        test_network = PlaceNetwork()
        test_data = loadNetwork("wps/wp_{}".format(i))
        test_network.loadFromFile(test_data)

        loss = wgt_mse(test_network, true_network, [0, 1, 4, 5])
        totalloss.append(loss)

plt.figure()
plt.title("MSE of Weights vs Final Weights")
plt.xlabel("Training Step")
plt.ylabel("MSE")
plt.xlim(0, 351)
plt.plot(updates, totalloss)
plt.show()

#pickle.dump(avgs, open("avgs.pkl", "wb"))


