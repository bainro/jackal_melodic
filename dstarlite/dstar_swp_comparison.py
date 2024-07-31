from dstar import DStarLite
import sys
sys.path.append('..')
from utils import loadNetwork, create_custom_cmap
from placecell import PlaceNetwork
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.colors import BoundaryNorm
import matplotlib.image as mpimg




def plotAdaptive(self, costmap=0, image=None, title="Cost Map", path=None, colorscale=plt.cm.Set1, obstacles=None):
    #fig, ax = plt.subplots(figsize=(12, 12))
    fig = plt.figure(figsize=(12, 12), dpi=900)
    ax = fig.add_axes([0.05, 0.05, 0.85, 0.85])
    ax.tick_params(left=False, right=False, labelleft=False, labelbottom=False, bottom=False)

    wgts = self.normalizeWeights(costmap)

    #Get colors
    colors = []

    for cell in self.cells:
        mean = []
        for con in self.cells:
            if (con.ID, cell.ID) in wgts.keys():
                mean.append(wgts[(con.ID, cell.ID)])
        colors.append(round(np.mean(mean)))

    ms = 15 #20
    colors = np.array(colors)
    cmap = create_custom_cmap()
    norm = Normalize(vmin=1.0, vmax=10.0)
    sm = ScalarMappable(cmap=cmap, norm=norm)
    color_vectors = sm.to_rgba(colors, alpha=None)

    bounds = [1, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5]#, 9.5, 10]
    norm = BoundaryNorm(bounds, cmap.N)

    ax_colorbar = fig.add_axes([0.925, 0.1, 0.03, 0.65])
    cbar = plt.colorbar(sm, cax=ax_colorbar)
    cbar = plt.colorbar(sm, cax=ax_colorbar, ticks=np.arange(0, 9), boundaries=bounds)
    cbar.ax.tick_params(labelsize=24)


    cellpairs = []

    for i, cell in enumerate(self.cells):
        ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms, color=color_vectors[i], zorder=20)

        for connected_cell in cell.connections.values():
            if (min(cell.ID, connected_cell.ID), max(cell.ID, connected_cell.ID)) in cellpairs:
                continue
            conncell = (self.points[connected_cell.ID][0], self.points[connected_cell.ID][1])
            ax.plot([self.points[cell.ID][1], conncell[1]], [self.points[cell.ID][0], conncell[0]],'ko-', zorder=1, linewidth=3.5, alpha=0.25)
            cellpairs.append((min(cell.ID, connected_cell.ID), max(cell.ID, connected_cell.ID)))

    if path is not None:

        used = []
        #Cell outlines
        for i, p in enumerate(path):
            for cell in self.cells:
                if cell.ID in p and cell.ID not in used:
                    used.append(cell.ID)
                    ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 8, color=colorscale(i), zorder=5+i)
                elif cell.ID in p and cell.ID in used:
                    ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 5, color=colorscale(i), zorder=5+i)

        #Path outlines
        for j, p in enumerate(path):
            for i in range(len(p) - 1):
                ax.plot([self.points[p[i]][1], self.points[p[i+1]][1]], [self.points[p[i]][0], self.points[p[i+1]][0]],'o-', color=colorscale(j), zorder=5, linewidth=8.0)

    if obstacles is not None:
        for (u, v) in obstacles:
            halfway = ((u[0] + v[0]) / 2, (u[1] + v[1]) / 2)
            ax.plot(halfway[1], halfway[0], '*', color='yellow', markersize=30, zorder=10, label="Placed Obstacles", markeredgecolor='black', markeredgewidth=2)

    ax.set_title(title, fontsize=30)

    if image is not None:
        background_image = mpimg.imread(image)
        square_x = 300   # Starting x-coordinate of the square
        square_y = 0   # Starting y-coordinate of the square
        square_size = 1400   # Size of the square (adjust as needed)

        # Extract the square portion from the rectangular image
        square_image = background_image[square_y:square_y+square_size, square_x:square_x+square_size]
        
        # Set the extent to fit the square plot while maintaining aspect ratio
        ax.imshow(square_image, extent=[-1, 17, -1, 17], aspect='auto', zorder=0, alpha=0.5)

    plt.margins(0)

def get_total_cost(network, costmaps, path):
    wgts = network.normalizeWeights(costmaps)

    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += wgts[(path[i], path[i + 1])]
    return total_cost

def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_matching_path(network):
    costmap = network.normalizeWeights([0, 1, 4, 5])
    neighbors = {network.points[cell.ID]: [network.points[neighbor.ID] for neighbor in cell.connections.values()] for cell in network.cells}
    node_list = [network.points[cell.ID] for cell in network.cells]

    for i in range(1000):
        # Randomly select start and goal
        start = node_list[np.random.randint(0, len(node_list))]
        goal = node_list[np.random.randint(0, len(node_list))]
        while(goal == start or euclidean_distance(start, goal) < 10):
            start = node_list[np.random.randint(0, len(node_list))]
            goal = node_list[np.random.randint(0, len(node_list))]
            
        dstar = DStarLite(start, goal, neighbors, node_list, network, costmap)
        dstar.compute_shortest_path()
        dstar_p = [network.points[p] for p in dstar.get_path()]

        sw_p = network.spikeWave(start, goal, costmap=[0, 1, 4, 5])
        sw_p = sw_p[::-1]

        if dstar_p == sw_p:
            print("Found matching path")
            print(start, goal)
            return start, goal
        
    print("No matching path found")
        
def adaptive_swp(network, start, goal, obstacles, costmap = [0, 1, 4, 5]):

    path_history = []

    costs = []
    path = [start]
    sw_p = network.spikeWave(start, goal, costmap=costmap)
    sw_p = [network.points[p] for p in sw_p[::-1]]
    path_history.append(sw_p)
    current_pos = start
    idx = 1
    while current_pos != goal:
        p = sw_p[idx]
        curr_ID = network.points[current_pos]
        next_ID = network.points[p]
        #print("Current pos:", current_pos)
        #print("Next pos:", p)

        if (current_pos, p) in obstacles:
            cost = network.cells[curr_ID].wgts[next_ID]
            cost[1] = 8.0
            costs.append(cost)
            #print("Path so far:", sw_p[:idx + 1])
            network.eProp(costs, sw_p[:idx + 1])

            costs = []
            sw_p = network.spikeWave(current_pos, goal, costmap=costmap)
            sw_p = [network.points[p] for p in sw_p[::-1]]
            path_history.append(sw_p)
            #print("New path:", sw_p)
            idx = 1
        else:
            costs.append(network.cells[curr_ID].wgts[next_ID])
            path.append(p)
            idx += 1
            current_pos = p

    return path, path_history

if __name__ == '__main__':
    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    #start, goal = find_matching_path(network)
    #exit()

    costmap = network.normalizeWeights([0, 1, 4, 5])
    id_to_point = {cell.ID: network.points[cell.ID] for cell in network.cells}
    neighbors = {network.points[cell.ID]: [network.points[neighbor.ID] for neighbor in cell.connections.values()] for cell in network.cells}

    node_list = [network.points[cell.ID] for cell in network.cells]
    start = (5, 11)
    goal = (15, 4)

    obstacle_placed = [((9, 8), (10, 8)), ((11, 8), (12, 7))]

    dstar = DStarLite(start, goal, neighbors, node_list, network, costmap)
    dstar_p = dstar.move_and_replan(start, obstacle_placed)
    dstar_p = [network.points[p] for p in dstar_p]

    sw_p, swp_path_history = adaptive_swp(network, start, goal, obstacle_placed)
    swp_path_history = [[network.points[p] for p in path] for path in swp_path_history]
    sw_p = [network.points[p] for p in sw_p]

    print(dstar_p)
    print(get_total_cost(network, [0, 1, 4, 5], sw_p))
    print(get_total_cost(network, [0, 1, 4, 5], dstar_p))

    plotAdaptive(network, costmap=[0, 1, 4, 5], image="../images/map/mapraw.jpg", title=None, path=[sw_p, dstar_p], obstacles=obstacle_placed)#, p1, p2])
    # Specify line colors and labels
    colors = ['tab:red', 'tab:blue', 'yellow']
    labels = ["SWP", "Dstar", 'Placed Obstacles']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=20, linestyle='None', markeredgecolor='black', markeredgewidth=2)

    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("dstarcomp.png")
    plt.close()
