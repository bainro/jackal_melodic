import sys
sys.path.append('../')
from utils import get_distance
from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import Any, Tuple
from placecell import PlaceNetwork, loadNetwork
from tqdm import tqdm
import pickle

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
        priority: int
        item: Any=field(compare=False)
        camefrom: Any=field(compare=False)
        
    open = PriorityQueue()
    visited = set()
    
    wgt_dict = network.normalizeWeights(costmap)
    startID = network.points[startPt[0], startPt[1]]
    goalID = network.points[goalPt[0], goalPt[1]]

    open.put(Item(item=startID, priority=0, camefrom=None))

    while not open.empty():
        
        current = open.get_nowait()

        if current.item == goalID:
            return getPath(current)

        visited.add(current.item)

        neighbors = [i.ID for i in network.cells[current.item].connections.values()]
        for neighborID in neighbors:
            if neighborID not in visited:
                cost = current.priority + round(wgt_dict[(current.item, neighborID)])
                open.put(Item(item=neighborID, priority=cost, camefrom=current))

    print("Goal couldn't be reached (shouldn't get here)")
    return None

network = PlaceNetwork()
data = loadNetwork("../fixed_wgts")
network.loadFromFile(data)

naive_network = PlaceNetwork()
naive_network.initAldritch(numcosts=6)
naive_network.initConnections()
#(15, 1) to (9, 11) for obstacles

rrt_p = network.RRTstar((15, 1), (9, 11), costmap=[0, 1, 4, 5])
sw_p = network.spikeWave((15, 1), (9, 11), costmap=[0, 1, 4, 5])
astar_p = astar(network, (15, 1), (9, 11), costmap=[0, 1, 4, 5])
naive_p = astar(naive_network, (15, 1), (9, 11), costmap=[0])

mindistance = 3

same = 0
total = 0

wps = []
st_ends = []

astar_pths = []
sw_pths = []
rrt_pths = []
naive_pths = []

for cell in network.cells:
    wps.append((network.points[cell.ID][0], network.points[cell.ID][1]))

for start in wps:
    for end in wps:
        if start == end or get_distance(start, end) < 2:
            continue
        st_ends.append((start, end))

for test_pt in tqdm(st_ends):
    astar_p = astar(network, test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])
    naive_p = astar(network, test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])
    sw_p = network.spikeWave(test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])
    rrt_p = network.RRTstar(test_pt[0], test_pt[1], costmap=[0, 1, 4, 5])

    astar_pths.append(astar_p)
    naive_pths.append(naive_p)
    sw_pths.append(sw_p)
    rrt_pths.append(rrt_p)

pickle.dump(astar_pths, open("astar_pths.pkl", "wb"))
pickle.dump(naive_pths, open("naive_pths.pkl", "wb"))
pickle.dump(sw_pths, open("sw_pths.pkl", "wb"))
pickle.dump(rrt_pths, open("rrt_pths.pkl", "wb"))

