from placecell import PlaceNetwork
from utils import loadNetwork
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

def adaptive_figure():
    matplotlib.rc('font', family='serif')
    initial_network = PlaceNetwork()
    initial_data = loadNetwork("chkpt")
    initial_network.loadFromFile(initial_data)

    p1_network = PlaceNetwork()
    p1_data = loadNetwork("adaptive/trial1")
    p1_network.loadFromFile(p1_data)

    p2_network = PlaceNetwork()
    p2_data = loadNetwork("adaptive/trial2")
    p2_network.loadFromFile(p2_data)

    p0 = initial_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    p1 = p1_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    p2 = p2_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5]) 

    p1_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=[p0, p1, p2], diff_map=True)#, p1, p2])
    # Specify line colors and labels
    #colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    labels = ["Original Path", "Path after 1st Update", "Path after 2nd Update", 'Placed Obstacle']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=20, linestyle='None',markeredgecolor='black', markeredgewidth=1)

    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("revisions/adaptive.pdf", dpi=600)
    plt.close()
    #plt.show()

def graphs():
    matplotlib.rc('font', family='serif')
    initial_network = PlaceNetwork()
    initial_data = loadNetwork("chkpt")
    initial_network.loadFromFile(initial_data)

    p1_network = PlaceNetwork()
    p1_data = loadNetwork("adaptive/trial1")
    p1_network.loadFromFile(p1_data)

    p2_network = PlaceNetwork()
    p2_data = loadNetwork("adaptive/trial2")
    p2_network.loadFromFile(p2_data)

    p0 = initial_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    p1 = p1_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    #p2 = p2_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])

    p1_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=[p0, p1], diff_map=True)#, p1, p2])
    # Specify line colors and labels
    #colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    colors = ['tab:red', 'tab:blue', 'yellow']
    labels = ["Original Path", "Path after 1st Update", 'Placed Obstacle']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=10, linestyle='None')

    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("video/pathstogether.png")
    plt.show()

    p1_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=[p0], diff_map=True)#, p1, p2])
    # Specify line colors and labels
    #colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    colors = ['tab:red', 'yellow']
    labels = ["Original Path", 'Placed Obstacle']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=10, linestyle='None')

    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("video/path1.png")
    plt.show()

    p1_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=[p1], diff_map=True)#, p1, p2])
    # Specify line colors and labels
    #colors = ['tab:red', 'tab:blue', 'tab:green', 'yellow']
    colors = ['tab:red', 'yellow']
    labels = ["Path after 1st Update", 'Placed Obstacle']
    dummy_lines = [Line2D([0], [0], color=color, linewidth=2) for color in colors]
    dummy_lines[-1] = Line2D([0], [0], marker='*', color='yellow', markersize=10, linestyle='None')

    # Add legend with dummy lines and labels
    plt.legend(dummy_lines, labels, bbox_to_anchor=(-3.0, 1.225), loc='upper right', fontsize=20)
    plt.savefig("video/path2.png")
    plt.close()

    #initial_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=None)
    #plt.savefig("initial.png")
    #plt.show()

    #p1_network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title=None, path=None)
    #plt.savefig("p1.png")
    #plt.show()

def get_total_cost(network, costmaps, path):
    wgts = network.normalizeWeights(costmaps)

    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += wgts[(path[i], path[i + 1])]
    return total_cost

if __name__ == '__main__':
    #adaptive_figure()
    #exit()
    matplotlib.rc('font', family='serif')
    initial_network = PlaceNetwork()
    initial_data = loadNetwork("chkpt")
    initial_network.loadFromFile(initial_data)

    p1_network = PlaceNetwork()
    p1_data = loadNetwork("adaptive/trial1")
    p1_network.loadFromFile(p1_data)

    p2_network = PlaceNetwork()
    p2_data = loadNetwork("adaptive/trial2")
    p2_network.loadFromFile(p2_data)

    p0 = initial_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    p1 = p1_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])
    p2 = p2_network.spikeWave((12, 11), (5, 11), costmap=[0, 1, 4, 5])

    #reverse the path
    p0 = p0[::-1]
    p1 = p1[::-1]
    p2 = p2[::-1]

    wgts_p0 = initial_network.normalizeWeights([0, 1, 4, 5])
    wgts_p1 = p1_network.normalizeWeights([0, 1, 4, 5])
    wgts_p2 = p2_network.normalizeWeights([0, 1, 4, 5])

    fig = plt.figure(figsize=(10, 10))
    #Two side by side subplots
    # Two side by side subplots


    width = 0.43
    height = 0.68

    buffer = 1 - 2*width -  0.1
    #print(buffer)
    spacing = buffer + width + 0.05

    ax1 = fig.add_subplot(121, aspect='equal', adjustable='box', position=[buffer, 0.30, width, height])
    ax2 = fig.add_subplot(122, aspect='equal', adjustable='box', position=[spacing, 0.30, width, height])


    #Get a color map for the changes
    cmap = plt.get_cmap('coolwarm')
    norm = Normalize(vmin=-1.0, vmax=1.0)
    sm = ScalarMappable(cmap=cmap, norm=norm)

    ax_colorbar = fig.add_axes([0.075, 0.22, 0.85, 0.03])
    cbar = plt.colorbar(sm, cax=ax_colorbar, orientation='horizontal')
    #Set font size 
    cbar.ax.tick_params(labelsize=20)

    #no ticks
    ax1.set_xticks([])
    ax1.set_yticks([])
    ax2.set_xticks([])
    ax2.set_yticks([])

    # Customizing the spines to have red dashed lines
    for spine in ax1.spines.values():
        spine.set_edgecolor('red')
        spine.set_linestyle('--')
        spine.set_linewidth(4)

    for spine in ax2.spines.values():
        spine.set_edgecolor('red')
        spine.set_linestyle('--')
        spine.set_linewidth(4)

    initial_network.plotChange(p1_network, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", ax=ax1)
    p1_network.plotChange(p2_network, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", ax=ax2)
    plt.savefig("revisions/weightchange_combined.pdf", dpi=900)
    plt.show()

    # print("Initial Path: ", p0)
    # print("Path after 1st Update: ", p1)
    # print("Path after 2nd Update: ", p2)

    # print("Initial Path Cost: ", get_total_cost(initial_network, [0, 1, 4, 5], p0))
    # print("Initial Path Cost: ", get_total_cost(initial_network, [0, 1, 4, 5], p1))
    # print("Initial Path Cost: ", get_total_cost(initial_network, [0, 1, 4, 5], p2))

    # print("Path after 1st Update Cost: ", get_total_cost(p1_network, [0, 1, 4, 5], p0))
    # print("Path after 1st Update Cost: ", get_total_cost(p1_network, [0, 1, 4, 5], p1))
    # print("Path after 1st Update Cost: ", get_total_cost(p1_network, [0, 1, 4, 5], p2))

    # print("Path after 2nd Update Cost: ", get_total_cost(p2_network, [0, 1, 4, 5], p0))
    # print("Path after 2nd Update Cost: ", get_total_cost(p2_network, [0, 1, 4, 5], p1))
    # print("Path after 2nd Update Cost: ", get_total_cost(p2_network, [0, 1, 4, 5], p2))