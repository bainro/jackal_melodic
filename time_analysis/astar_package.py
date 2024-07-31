from astar import AStar
import math
from dataclasses import dataclass, field
from typing import Any
import sys
sys.path.append("..")

from utils import loadNetwork, haversineDistance
from placecell import PlaceNetwork

class NetworkAStar(AStar):
    def __init__(self, network, costmap):
        super().__init__()
        self.network = network
        self.wgt_dict = self.network.normalizeWeights(costmap)
        
    def heuristic_cost_estimate(self, start, goal):
        #print(f"Estimate from {start} to {goal}: {self.get_distance(self.network.points[start], self.network.points[goal])}")
        return self.get_distance(self.network.points[start], self.network.points[goal])

    def distance_between(self, n1, n2):
        #print(f"Distance between {n1} and {n2}: {round(self.wgt_dict[(n1, n2)])}")
        return round(self.wgt_dict[(n1, n2)])

    def neighbors(self, node):
        #print(f"Neighbors of {node}: {[i.ID for i in self.network.cells[node].connections.values()]}")
        return [i.ID for i in self.network.cells[node].connections.values()]

    def get_distance(self, pt1, pt2):
        #print(f"Distance between {pt1} and {pt2}: {math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)}")
        return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)
    
    def is_goal_reached(self, current: Any, goal: Any):
        if current == goal:
            return True
        else:
            return False

    def getPath(self, startPt, goalPt):
        startID = self.network.points[startPt[0], startPt[1]]
        goalID = self.network.points[goalPt[0], goalPt[1]]
        #print(startID, goalID)
        path = self.astar(startID, goalID)
        return list(path) if path else None

def astar_analysis_package(paths, solver):
    for path in paths:
        solver.getPath(path[0], path[1])

# Usage
def astar_test(network, startPt, goalPt, costmap):
    astar_solver = NetworkAStar(network, costmap)
    return astar_solver.getPath(startPt, goalPt)

if __name__ == "__main__":
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    startPt = (0, 0)
    goalPt = (5, 5)
    print(astar_test(network, startPt, goalPt, [0, 1, 4, 5]))