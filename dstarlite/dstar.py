import sys
import math
sys.path.append('..')
from utils import loadNetwork
from placecell import PlaceNetwork
from priority_queue import PriorityQueue, Priority


def heuristic(p, q) -> float:
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

class DStarLite:
    def __init__(self, s_start, s_goal, neighbors, node_list, network, costmap):
        self.network = network
        self.neighbors = neighbors
        self.costmap = costmap

        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0

        self.g = {}
        self.rhs = {}
        for node in node_list:
            self.g[node] = float('inf')
            self.rhs[node] = float('inf')
        self.rhs[s_goal] = 0
        self.U = PriorityQueue()
        
        self.U.insert(self.s_goal, Priority(heuristic(self.s_start, self.s_goal), 0))

    def calculate_key(self, s):
        k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)
    
    def c(self, u, v):
        return round(self.costmap[(self.network.points[u], self.network.points[v])])
    
    def contains(self, u):
        return u in self.U.vertices_in_heap
    
    def update_vertex(self, u):
        if self.g[u] != self.rhs[u] and self.contains(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g[u] != self.rhs[u] and not self.contains(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u] == self.rhs[u] and self.contains(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

           # print(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                for s in self.neighbors[u]:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.c(s, u) + self.g[u])
                    self.update_vertex(s)
            else:
                self.g_old = self.g[u]
                self.g[u] = float('inf')
                pred = self.neighbors[u]
                pred.append(u)
                for s in pred:
                    if s != u and self.rhs[s] == self.c(s, u) + self.g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            succ = self.neighbors[s]
                            for s_ in succ:
                                temp = self.c(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s] = min_s
                    self.update_vertex(s)

    def move_and_replan(self, start, obstacles):
        #Obstacles come in the form of a list of tuples, where each tuple is a pair of points
        # Ex. [((0,1),(0,2)), ((1,1),(1,2)), ...]
        path = [start]
        self.s_start = start
        self.s_last = self.s_start
        self.compute_shortest_path()

        while self.s_start != self.s_goal:
            assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"

            succ = self.neighbors[self.s_start]
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.c(self.s_start, s_) + self.g[s_]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_
            

            for (u, v) in obstacles:
                if (self.s_start == u and arg_min == v) or (self.s_start == v and arg_min == u):

                    u_id = self.network.points[u]
                    v_id = self.network.points[v]

                    self.costmap[(u_id, v_id)] = 8.0
                    self.costmap[(v_id, u_id)] = 8.0
                    self.update_vertex(u)
                    self.update_vertex(v)
                    self.k_m += heuristic(self.s_last, self.s_start)
                    self.s_last = self.s_start
                    self.compute_shortest_path()
                    
                    succ = self.neighbors[self.s_start]
                    min_s = float('inf')
                    arg_min = None
                    for s_ in succ:
                        temp = self.c(self.s_start, s_) + self.g[s_]
                        if temp < min_s:
                            min_s = temp
                            arg_min = s_

            self.s_start = arg_min
            path.append(self.s_start)

        return path
            


    def get_path(self):

        path = [self.s_start]
        self.s_last = self.s_start

        while self.s_start != self.s_goal:
            assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"

            succ = self.neighbors[self.s_start]
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.c(self.s_start, s_) + self.g[s_]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_
            
            self.s_start = arg_min
            path.append(self.s_start)

        return path

if __name__ == '__main__':
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    costmap = network.normalizeWeights([0, 1, 4, 5])
    id_to_point = {cell.ID: network.points[cell.ID] for cell in network.cells}
    neighbors = {network.points[cell.ID]: [network.points[neighbor.ID] for neighbor in cell.connections.values()] for cell in network.cells}

    node_list = [network.points[cell.ID] for cell in network.cells]
    start = (0, 12)
    goal = (12, 12)

    dstar = DStarLite(start, goal, neighbors, node_list, network, costmap)
    dstar.compute_shortest_path()
    print(dstar.get_path())
