from dataclasses import dataclass, field
from queue import PriorityQueue
from typing import Any
import random
import numpy as np

#Add parent directory to path
import sys
sys.path.append("..")

from utils import loadNetwork, haversineDistance
from placecell import PlaceNetwork

import timeit
import pickle
import os
import matplotlib.pyplot as plt
from tqdm import tqdm


def get_distance(pt1, pt2):
    return ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**0.5

def astar(network, startPt, goalPt, costmap):

    def getPath(end):
        path = []
        current = end
        while current.camefrom is not None:
            path.append(current.item)
            current = current.camefrom

        path.append(current.item)
        return path
    
    @dataclass(order=True)
    class Item:
        priority: float
        distance_tie: float
        item: Any=field(compare=False)
        camefrom: Any=field(compare=False)
        
    open = PriorityQueue()
    visited = set()
    
    wgt_dict = network.normalizeWeights(costmap)
    startID = network.points[startPt[0], startPt[1]]
    goalID = network.points[goalPt[0], goalPt[1]]

    open.put_nowait(Item(item=startID, priority=0, distance_tie=0.0, camefrom=None))

    while not open.empty():
        
        current = open.get_nowait()

        if current.item == goalID:
            return getPath(current)

        visited.add(current.item)

        neighbors = [i.ID for i in network.cells[current.item].connections.values()]
        for neighborID in neighbors:
            if neighborID not in visited:
                cost = current.priority + round(wgt_dict[(current.item, neighborID)])
                heuristic = get_distance(network.points[neighborID], network.points[goalID])
                open.put_nowait(Item(item=neighborID, priority=cost + heuristic, distance_tie=get_distance(network.points[neighborID], network.points[goalID]), camefrom=current))

    print("Goal couldn't be reached (shouldn't get here)")
    return None

def astar_analysis(paths, network):
    for path in paths:
        astar(network, path[0], path[1], [0, 1, 4, 5])

def spikewave_analysis(paths, network):
    for path in paths:
        network.spikeWave(path[0], path[1], costmap=[0, 1, 4, 5])

def rrt_analysis(paths, network):
    for path in paths:
        network.RRTstar(path[0], path[1], costmap=[0, 1, 4, 5])

def run_n_for_max_distance(network, n, max_distance):
    wps = []
    st_ends = []
    for cell in network.cells:
        wps.append((network.points[cell.ID][0], network.points[cell.ID][1]))

    for start in wps:
        for end in wps:
            if start == end or get_distance(start, end) > max_distance:
                continue
            st_ends.append((start, end))

    astar_times = []
    spikewave_times = []
    rrt_times = []

    #Run each n times
    for i in tqdm(range(n)):
        #print("Iteration: ", i)
        random.shuffle(st_ends)
        subset = st_ends[:500]
        time_astar = timeit.timeit(lambda: astar_analysis(subset, network), number=1)
        time_spikewave = timeit.timeit(lambda: spikewave_analysis(subset, network), number=1)
        time_rrst = timeit.timeit(lambda: rrt_analysis(subset, network), number=1)

        astar_times.append(time_astar)
        spikewave_times.append(time_spikewave)
        rrt_times.append(time_rrst)

    #Get mean and std dev
    astar_mean = np.mean(astar_times)
    astar_std = np.std(astar_times)
    spikewave_mean = np.mean(spikewave_times)
    spikewave_std = np.std(spikewave_times)
    rrt_mean = np.mean(rrt_times)
    rrt_std = np.std(rrt_times)

    data = {"astar": (astar_mean, astar_std), "spikewave": (spikewave_mean, spikewave_std), "rrt": (rrt_mean, rrt_std)}

    return data




if __name__ == "__main__":
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    naive_network = PlaceNetwork()
    naive_network.initAldritch(numcosts=6)
    naive_network.initConnections()

    if not os.path.exists("time_analysis.pkl"):
        #Distances from 1 to 10
        distances = [i for i in range(1, 21)]
        data = {}
        data["naive"] = {}
        data["learned"] = {}
        for d in distances:
            data["naive"][d] = run_n_for_max_distance(naive_network, n=5, max_distance=d)
            data["learned"][d] = run_n_for_max_distance(network, n=5, max_distance=d)

        with open("time_analysis.pkl", "wb") as f:
            pickle.dump(data, f)
    else:
        with open("time_analysis.pkl", "rb") as f:
            data = pickle.load(f)

    #Plotting
    distances = [i for i in range(1, 11)]
    naive_astar_means = [data["naive"][d]["astar"][0] for d in distances]
    naive_astar_std = [data["naive"][d]["astar"][1] for d in distances]
    learned_astar_means = [data["learned"][d]["astar"][0] for d in distances]
    learned_astar_std = [data["learned"][d]["astar"][1] for d in distances]

    naive_spikewave_means = [data["naive"][d]["spikewave"][0] for d in distances]
    naive_spikewave_std = [data["naive"][d]["spikewave"][1] for d in distances]
    learned_spikewave_means = [data["learned"][d]["spikewave"][0] for d in distances]
    learned_spikewave_std = [data["learned"][d]["spikewave"][1] for d in distances]

    naive_rrt_means = [data["naive"][d]["rrt"][0] for d in distances]
    naive_rrt_std = [data["naive"][d]["rrt"][1] for d in distances]
    learned_rrt_means = [data["learned"][d]["rrt"][0] for d in distances]
    learned_rrt_std = [data["learned"][d]["rrt"][1] for d in distances]

    #Plot means as line
    plt.figure()
    #lt.plot(distances, naive_astar_means, label="Naive A*")
    plt.plot(distances, learned_astar_means, label="Learned A*")
    #plt.plot(distances, naive_spikewave_means, label="Naive SpikeWave")
    plt.plot(distances, learned_spikewave_means, label="Learned SpikeWave")
    #plt.plot(distances, naive_rrt_means, label="Naive RRT")
    plt.plot(distances, learned_rrt_means, label="Learned RRT")
    plt.xlabel("Max Distance")
    plt.ylabel("Time (s)")
    plt.legend()
    plt.show()

