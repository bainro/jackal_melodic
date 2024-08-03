import heapq
import math
import sys
from dstarlite import dstar_adaptive, dstar_plan
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize, BoundaryNorm
from matplotlib.lines import Line2D
import numpy as np
from tqdm import tqdm
import scipy.stats as stats

sys.path.append('../')
from utils import loadNetwork, create_custom_cmap, get_distance
from placecell import PlaceNetwork

def one_tailed_t_test(array1, array2, alternative='greater'):
    """
    Perform a one-tailed t-test on two arrays.

    Parameters:
    - array1, array2: The two arrays for which the t-test will be performed.
    - alternative: The direction of the test. It can be 'greater' or 'less'.
    
    Returns:
    - t_statistic: The t-statistic of the test.
    - p_value: The p-value of the test.
    """
    _, p_value = stats.ttest_ind(array1, array2)
    
    if alternative == 'greater':
        t_statistic = p_value / 2  # Divide p-value by 2 for a one-tailed test
    elif alternative == 'less':
        t_statistic = 1 - p_value / 2  # Subtract p-value divided by 2 from 1
    else:
        raise ValueError("Invalid alternative. Use 'greater' or 'less'.")

    result = {
        'statistic': t_statistic,
        'p_value': p_value,
    }
    
    return result

def find_matching_path(network):
    node_list = [network.points[cell.ID] for cell in network.cells]

    for i in range(1000):
        # Randomly select start and goal
        start = node_list[np.random.randint(0, len(node_list))]
        goal = node_list[np.random.randint(0, len(node_list))]
        while(goal == start or get_distance(start, goal) < 10):
            start = node_list[np.random.randint(0, len(node_list))]
            goal = node_list[np.random.randint(0, len(node_list))]
            
        dstar_p = dstar_plan(network, start, goal)
        dstar_p = [network.points[p] for p in dstar_p]

        sw_p = network.spikeWave(start, goal, costmap=[0, 1, 4, 5])
        sw_p = sw_p[::-1]

        if dstar_p == sw_p:
            #print("Found matching path")
            return start, goal, dstar_p
        
    print("No matching path found")
    return None, None
        

def plotAdaptive(self, costmap=0, image=None, title="Cost Map", path=None, colorscale=plt.cm.Set1, obstacles=None):
    #fig, ax = plt.subplots(figsize=(12, 12))
    fig = plt.figure(figsize=(12, 12), dpi=300)
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
                    #Make 2nd transparent
                    if i == 2:
                        ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 8, color=colorscale(i), zorder=5+i, alpha=0.5)
                    else:
                        ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 8, color=colorscale(i), zorder=5+i)
                elif cell.ID in p and cell.ID in used:
                    if i == 2:
                        ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 5, color=colorscale(i), zorder=5+i, alpha=0.5)
                    else:
                        ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms + 5, color=colorscale(i), zorder=5+i)

        #Path outlines
        for j, p in enumerate(path):
            for i in range(len(p) - 1):
                if j == 2:
                    ax.plot([self.points[p[i]][1], self.points[p[i+1]][1]], [self.points[p[i]][0], self.points[p[i+1]][0]],'o-', color=colorscale(j), zorder=5, linewidth=8.0, alpha=0.5)
                else:
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

def adaptive_swp(network, start, goal, obstacles, costmap = [0, 1, 4, 5]):
    timeout = 0
    path_history = []

    costs = []
    path = [start]
    sw_p = network.spikeWave(start, goal, costmap=costmap)
    original_path = sw_p
    sw_p = [network.points[p] for p in sw_p[::-1]]
    path_history.append(sw_p)
    current_pos = start
    idx = 1

    obstacles_seen = []
    hit_obstacle = False
    while current_pos != goal:
        timeout += 1
        if timeout > 100:
            print("Timeout")
            return None, None
        p = sw_p[idx]
        curr_ID = network.points[current_pos]
        next_ID = network.points[p]
        #print("Current pos:", current_pos)
        #print("Next pos:", p)

        if (current_pos, p) in obstacles:
            if (current_pos, p) in obstacles_seen:
                hit_obstacle = True
                #print("Obstacle detected again at ", (current_pos, p))
                costs.append(network.cells[curr_ID].wgts[next_ID])
                path.append(p)
                idx += 1
                current_pos = p
            else:
                obstacles_seen.append((current_pos, p))
                #print("Obstacle detected at ", (current_pos, p))
                cost = network.cells[curr_ID].wgts[next_ID]
                cost[1] = 8.0
                cost[5] = 10.0
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

    return path, hit_obstacle

def get_total_cost(network, costmaps, path):
    wgts = network.normalizeWeights(costmaps)

    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += wgts[(path[i], path[i + 1])]
    return total_cost

def n_trials_matching_paths(n):
    same_paths = 0

    swp_paths = []
    dstar_paths = []

    used_start_ends = []
    for i in tqdm(range(n)):
        network = PlaceNetwork()
        data = loadNetwork("../fixed_wgts")
        network.loadFromFile(data)

        start, end, path = find_matching_path(network)
        while (start, end) in used_start_ends:
            start, end, path = find_matching_path(network)
        used_start_ends.append((start, end))

        coord_path = [network.points[p] for p in path]
        k_m = 0
        s_start = start
        s_goal = end
        obstacle = [(coord_path[len(coord_path) // 2],coord_path[(len(coord_path) // 2) + 1])]

        dstar_p = dstar_adaptive(network, s_start, s_goal, obstacle)
        dstar_p = [network.points[p] for p in dstar_p]

        sw_p, hit = adaptive_swp(network, s_start, s_goal, obstacle)
        sw_p = [network.points[p] for p in sw_p]

        if dstar_p == sw_p:
            same_paths += 1
        if not hit:
            obstaclestart = network.points[obstacle[0][0]]
            obstacleend = network.points[obstacle[0][1]]
            network.cells[obstaclestart].wgts[obstacleend][1] = 8.0
            dstar_paths.append(get_total_cost(network, [0, 1, 4, 5], dstar_p))
            swp_paths.append(get_total_cost(network, [0, 1, 4, 5], sw_p))


    print("Number of matching paths:", same_paths)
    print("Mean cost of D* paths:", np.mean(dstar_paths))
    print("Mean cost of SWP paths:", np.mean(swp_paths))

    t_test = one_tailed_t_test(dstar_paths, swp_paths, alternative='greater')
    print("T-test result:", t_test)




if __name__ == '__main__':

    n_trials_matching_paths(1000)
    exit()

    network = PlaceNetwork()
    data = loadNetwork("../fixed_wgts")
    network.loadFromFile(data)

    start, end, path = find_matching_path(network)
    coord_path = [network.points[p] for p in path]
    k_m = 0
    s_start = start#(12, 14)
    s_goal = end#(1, 6)
    obstacle = [(coord_path[len(coord_path) // 2],coord_path[(len(coord_path) // 2) + 1])]#[((9, 12), (8, 11))]

    #path = network.spikeWave(s_start, s_goal, costmap=[0, 1, 4, 5])[::-1]

    dstar_p = dstar_adaptive(network, s_start, s_goal, obstacle)
    print(dstar_p)
    dstar_p = [network.points[p] for p in dstar_p]

    sw_p, _ = adaptive_swp(network, s_start, s_goal, obstacle)
    sw_p = [network.points[p] for p in sw_p]

    plotAdaptive(network, costmap=[0, 1, 4, 5], path=[sw_p, dstar_p, path], obstacles=obstacle, image="../images/map/mapraw.jpg")
        # Specify line colors and labels
    colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    labels = ["SWP (Ours)", "D* Adjustment", "Original Path", 'Placed Obstacles']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=20, linestyle='None', markeredgecolor='black', markeredgewidth=2)


    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("dstar_adaptive.png")
    #plt.show()