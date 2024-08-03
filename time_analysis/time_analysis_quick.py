from dataclasses import dataclass, field
from queue import PriorityQueue
from typing import Any
import random
import numpy as np

from astar_package import NetworkAStar, astar_analysis_package
#Add parent directory to path
import sys
sys.path.append("..")

from dstar_package.dstarlite import dstar_plan

from utils import loadNetwork, haversineDistance
from placecell import PlaceNetwork
import matplotlib

import timeit
import pickle
import os
import matplotlib.pyplot as plt
from tqdm import tqdm
import astar as Astar


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

def dstar_analysis(paths, network):
    for path in paths:
        dstar_plan(network, path[0], path[1])

def run_n_for_max_distance(network, n, min_distance):
    wps = []
    st_ends = []
    for cell in network.cells:
        wps.append((network.points[cell.ID][0], network.points[cell.ID][1]))

    for start in wps:
        for end in wps:
            start_origin = network.cells[network.points[start]].origin
            end_origin = network.cells[network.points[end]].origin
            if start == end or haversineDistance(start_origin[0], start_origin[1], end_origin[0], end_origin[1]) < min_distance:
                continue
            st_ends.append((start, end))

    print(f"Number of paths: {len(st_ends)} for distance {min_distance}")

    if st_ends == []:
        data = {"astar": (0, 0), "spikewave": (0, 0), "rrt": (0, 0)}
        return data

    dstar_times = []
    astar_times = []
    spikewave_times = []
    rrt_times = []

    astar_network = NetworkAStar(network, [0, 1, 4, 5])

    #Run each n times
    for i in tqdm(range(n)):
        #Shuffle and get subset of 500
        random.shuffle(st_ends)
        subset = st_ends[:1000]

        times_dstar = timeit.timeit(lambda: dstar_analysis(subset, network), number=1)
        time_astar = timeit.timeit(lambda: astar_analysis_package(subset, astar_network), number=1)
        time_spikewave = timeit.timeit(lambda: spikewave_analysis(subset, network), number=1)
        time_rrst = timeit.timeit(lambda: rrt_analysis(subset, network), number=1)

        dstar_times.append(times_dstar)
        astar_times.append(time_astar)
        spikewave_times.append(time_spikewave)
        rrt_times.append(time_rrst)

    #Get mean and std dev
    dstar_mean = np.mean(dstar_times)
    dstar_std = np.std(dstar_times)
    astar_mean = np.mean(astar_times)
    astar_std = np.std(astar_times)
    spikewave_mean = np.mean(spikewave_times)
    spikewave_std = np.std(spikewave_times)
    rrt_mean = np.mean(rrt_times)
    rrt_std = np.std(rrt_times)

    data = {"dstar": (dstar_mean, dstar_std), "astar": (astar_mean, astar_std), "spikewave": (spikewave_mean, spikewave_std), "rrt": (rrt_mean, rrt_std)}

    return data




if __name__ == "__main__":
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    naive_network = PlaceNetwork()
    naive_network.initAldritch(numcosts=6)
    naive_network.initConnections()

    if not os.path.exists("time_analysis_quick.pkl"):
        #Distances from 0 to 80 every 10
        distances = np.arange(0, 81, 10)
        data = {}
        data["learned"] = {}
        for d in distances:
            data["learned"][d] = run_n_for_max_distance(network, n=10, min_distance=d)

        with open("time_analysis_quick.pkl", "wb") as f:
            pickle.dump(data, f)
    else:
        with open("time_analysis_quick.pkl", "rb") as f:
            data = pickle.load(f)


    matplotlib.rc('font', family='serif')
    #Plotting
    distances = [i for i in np.arange(0, 81, 10)]

    learned_dstar_means = [data["learned"][d]["dstar"][0] for d in distances]
    learned_dstar_std = [data["learned"][d]["dstar"][1] for d in distances]

    learned_astar_means = [data["learned"][d]["astar"][0] for d in distances]
    learned_astar_std = [data["learned"][d]["astar"][1] for d in distances]

    learned_spikewave_means = [data["learned"][d]["spikewave"][0] for d in distances]
    learned_spikewave_std = [data["learned"][d]["spikewave"][1] for d in distances]

    learned_rrt_means = [data["learned"][d]["rrt"][0] for d in distances]
    learned_rrt_std = [data["learned"][d]["rrt"][1] for d in distances]

    #Plot means as line
    plt.figure(figsize=(12,12), dpi=900)
    plt.plot(distances, learned_astar_means, label="A*", color="tab:blue")
    plt.fill_between(distances, np.array(learned_astar_means) - np.array(learned_astar_std), np.array(learned_astar_means) + np.array(learned_astar_std), alpha=0.2, color="tab:blue")

    plt.plot(distances, learned_spikewave_means, label="SWP", color="tab:orange")
    plt.fill_between(distances, np.array(learned_spikewave_means) - np.array(learned_spikewave_std), np.array(learned_spikewave_means) + np.array(learned_spikewave_std), alpha=0.2, color="tab:orange")

    plt.plot(distances, learned_rrt_means, label="RRT*", color="tab:red")
    plt.fill_between(distances, np.array(learned_rrt_means) - np.array(learned_rrt_std), np.array(learned_rrt_means) + np.array(learned_rrt_std), alpha=0.2, color="tab:red")

    plt.plot(distances, learned_dstar_means, label="D* Lite", color="tab:purple")
    plt.fill_between(distances, np.array(learned_dstar_means) - np.array(learned_dstar_std), np.array(learned_dstar_means) + np.array(learned_dstar_std), alpha=0.2, color="tab:purple")

    plt.margins(0)

    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(fontsize=24)

    plt.xlabel("Minimum path length considered (meters)", fontsize=30)
    plt.ylabel("Time (s)", fontsize=30)
    plt.legend(fontsize=24, loc="upper left")
    plt.savefig("time_analysis_quick.png")

