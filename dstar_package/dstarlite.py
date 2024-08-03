import heapq
import math
import sys

sys.path.append('../')
from utils import loadNetwork
from placecell import PlaceNetwork

def euclidean(p, q) -> float:
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

class GraphNode:
    def __init__(self, id):
        self.id = id
        self.g = float('inf')
        self.rhs = float('inf')
        self.parents = {}
        self.children = {}

class Graph:
    def __init__(self, network, goal):
            self.network = network
            self.costmap = network.normalizeWeights([0, 1, 4, 5])
            neighbors = {network.points[cell.ID]: [network.points[neighbor.ID] for neighbor in cell.connections.values()] for cell in network.cells}
            node_list = [network.points[cell.ID] for cell in network.cells]

            self.goal = goal

            self.graph = {}
            for node in node_list:
                self.graph[node] = GraphNode(node)

            for node in node_list:
                for neighbor in neighbors[node]:
                    n = network.points[node]
                    _n = network.points[neighbor]
                    self.graph[node].children[neighbor] = round(self.costmap[(n, _n)])
                    self.graph[node].parents[neighbor] = round(self.costmap[(_n, n)])


def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]

def topKey(queue):
    queue.sort()
    # print(queue)
    if len(queue) > 0:
        return queue[0][:2]
    else:
        # print('empty queue!')
        return (float('inf'), float('inf'))


def heuristic_from_s(id, s):
    return euclidean(id, s)


def calculateKey(graph, id, s_current, k_m):
    return (min(graph.graph[id].g, graph.graph[id].rhs) + heuristic_from_s(id, s_current) + k_m, min(graph.graph[id].g, graph.graph[id].rhs))


def updateVertex(graph, queue, id, s_current, k_m):
    s_goal = graph.goal
    if id != s_goal:
        min_rhs = float('inf')
        for i in graph.graph[id].children:
            min_rhs = min(
                min_rhs, graph.graph[i].g + graph.graph[id].children[i])
        graph.graph[id].rhs = min_rhs
    id_in_queue = [item for item in queue if id == item[2]]
    if id_in_queue != []:
        if len(id_in_queue) != 1:
            raise ValueError('more than one ' + id + ' in the queue!')
        queue.remove(id_in_queue[0])
    if graph.graph[id].rhs != graph.graph[id].g:
        heapq.heappush(queue, calculateKey(graph, id, s_current, k_m) + (id,))


def computeShortestPath(graph, queue, s_start, k_m):
    while (graph.graph[s_start].rhs != graph.graph[s_start].g) or (topKey(queue) < calculateKey(graph, s_start, s_start, k_m)):
        k_old = topKey(queue)
        u = heapq.heappop(queue)[2]
        if k_old < calculateKey(graph, u, s_start, k_m):
            heapq.heappush(queue, calculateKey(graph, u, s_start, k_m) + (u,))
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for i in graph.graph[u].parents:
                updateVertex(graph, queue, i, s_start, k_m)
        else:
            graph.graph[u].g = float('inf')
            updateVertex(graph, queue, u, s_start, k_m)
            for i in graph.graph[u].parents:
                updateVertex(graph, queue, i, s_start, k_m)
        # graph.printGValues()


def nextInShortestPath(graph, s_current):
    min_rhs = float('inf')
    s_next = None
    if graph.graph[s_current].rhs == float('inf'):
        print('You are done stuck')
    else:
        for i in graph.graph[s_current].children:
            # print(i)
            child_cost = graph.graph[i].g + graph.graph[s_current].children[i]
            # print(child_cost)
            if (child_cost) < min_rhs:
                min_rhs = child_cost
                s_next = i
        if s_next:
            return s_next
        else:
            raise ValueError('could not find child for transition!')


def moveAndRescan(graph, queue, s_current, obstacles, k_m):
    if(s_current == graph.goal):
        return 'goal', k_m
    else:
        s_last = s_current
        s_new = nextInShortestPath(graph, s_current)

        #print('moving from ', s_current, ' to ', s_new)
        if((s_current, s_new) in obstacles):  # just ran into new obstacle
            #print('found obstacle at ', s_new)
            graph.graph[s_current].children[s_new] = 8.0
            graph.graph[s_new].children[s_current] = 8.0
            updateVertex(graph, queue, s_current, s_current, k_m)
            s_new = s_current 

        #results = scanForObstacles(graph, queue, s_last, s_new, obstacles, k_m)
        k_m += heuristic_from_s(s_last, s_new)
        computeShortestPath(graph, queue, s_current, k_m)

        return s_new, k_m


def initDStarLite(graph, queue, s_start, s_goal, k_m):
    graph.graph[s_goal].rhs = 0
    heapq.heappush(queue, calculateKey(
        graph, s_goal, s_start, k_m) + (s_goal,))
    computeShortestPath(graph, queue, s_start, k_m)

    return (graph, queue, k_m)

def dstar_adaptive(network, start, goal, obstacles):
    k_m = 0
    s_start = start
    s_goal = goal
    queue = []

    graph = Graph(network, s_goal)
    graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)

    s_current = s_start
    path = [s_current]
    while (s_current != s_goal):
        s_new, k_m = moveAndRescan(graph, queue, s_current, obstacles, k_m)
        if s_new == 'goal':
            break
        if s_new != s_current:
            path.append(s_new)
        s_current = s_new
    return path

def dstar_plan(network, start, goal):
    k_m = 0
    s_start = start
    s_goal = goal
    queue = []

    graph = Graph(network, s_goal)
    graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)

    s_current = s_start
    path = [s_current]
    while (s_current != s_goal):
        s_new = nextInShortestPath(graph, s_current)
        path.append(s_new)
        s_current = s_new
    return path

if __name__ == '__main__':
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    k_m = 0
    s_start = (12, 1)
    s_goal = (5, 11)
    print(dstar_plan(network, s_start, s_goal))


    obstacle = [((9, 5), (10, 6))]
    print(dstar_adaptive(network, s_start, s_goal, obstacle))