import sys
sys.path.append('../')
from utils import get_distance
sys.path.append('../dstarlite')
from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import Any, Tuple
import matplotlib.pyplot as plt
from placecell import PlaceNetwork, loadNetwork
from tqdm import tqdm
import numpy as np
import pickle
from dstarlite.dstar import DStarLite

sys.path.append('../dstar_package')
from dstar_package.dstarlite import dstar_plan

sys.path.append('../time_analysis')
from astar_package import NetworkAStar

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

network = PlaceNetwork()
data = loadNetwork("../fixed_wgts")
network.loadFromFile(data)

naive_network = PlaceNetwork()
naive_network.initAldritch(numcosts=6)
naive_network.initConnections()
#(15, 1) to (9, 11) for obstacles


# test_start = (0, 0)
# test_end = (10, 10)
# test_astar = astar(naive_network, test_start, test_end, costmap=[0, 1, 4, 5])
# test_sw = naive_network.spikeWave(test_start, test_end, costmap=[0, 1, 4, 5])

# print(test_astar)
# print(test_sw)

# naive_network.plotPath(test_astar, title='A*')
# naive_network.plotPath(test_sw, title='Spike Wave')
# plt.show()

# exit()



mindistance = 2

wps = []
st_ends = []

astar_pths = []
sw_pths = []
rrt_pths = []
naive_pths = []
dstar_pths = []

astar_other_pths = []

for cell in network.cells:
    wps.append((network.points[cell.ID][0], network.points[cell.ID][1]))

for start in wps:
    for end in wps:
        if start == end or get_distance(start, end) < mindistance:
            continue
        st_ends.append((start, end))

np.random.shuffle(st_ends)
total = 0
matched = 0

print("number of paths: ", len(st_ends))
#exit()

costmap = network.normalizeWeights([0, 1, 4, 5])
neighbors = {network.points[cell.ID]: [network.points[neighbor.ID] for neighbor in cell.connections.values()] for cell in network.cells}
node_list = [network.points[cell.ID] for cell in network.cells]

for test_pt in tqdm(st_ends):
    #astar_p = astar(network, test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])
    #naive_p = astar(naive_network, test_pt[0], test_pt[1], costmap=[0])
    #sw_p = network.spikeWave(test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])
    #rrt_p = network.RRTstar(test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])

    #astar_pths.append(astar_p)
    #naive_pths.append(naive_p)
    #sw_pths.append(sw_p)
    #rrt_pths.append(rrt_p)
    #dstar = DStarLite(test_pt[0], test_pt[1], neighbors, node_list, network, costmap)
    #dstar.compute_shortest_path()
    #dstar_p = dstar.get_path()
    #dstar_pths.append(dstar_p)

    dstar_pths.append(dstar_plan(network, test_pt[0], test_pt[1]))


#pickle.dump(astar_pths, open("astar_pths.pkl", "wb"))
#pickle.dump(naive_pths, open("naive_pths.pkl", "wb"))
#pickle.dump(sw_pths, open("sw_pths.pkl", "wb"))
#pickle.dump(rrt_pths, open("rrt_pths.pkl", "wb"))
pickle.dump(dstar_pths, open("dstar_pths.pkl", "wb"))
#pickle.dump(astar_other_pths, open("astar_other_pths.pkl", "wb"))