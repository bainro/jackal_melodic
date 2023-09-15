import numpy as np
import matplotlib.pyplot as plt
from placecell import PlaceCell, PlaceNetwork
from utils import *

def singleSpikeTest():
    network = PlaceNetwork()
    network.initAldritch()
    network.initConnections()

    path = network.spikeWave(0, 288)
    costs = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

    network.eProp(costs, path)

    #Print reversed path
    print(path[::-1])

    #Check if weights are updated correctly
    for cell in network.cells:
        mean = np.mean(list(cell.wgts.values()))
        if mean > 5:
            print(cell.ID, mean)
    network.plotCells()
    plt.show()

def saveAndReloadTest():
    network = PlaceNetwork()
    network.initAldritch()
    network.initConnections()
    path = network.spikeWave(0, 288)
    costs = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
    network.eProp(costs, path)

    #Save network
    saveNetwork(network, "test")

    #delete network
    del network

    #Reload network
    network = PlaceNetwork()
    data = loadNetwork("test")
    network.loadFromFile(data)
    #Check if weights are updated correctly
    for cell in network.cells:
        mean = np.mean(list(cell.wgts.values()))
        if mean > 5:
            print(cell.ID, mean)
    network.plotCells()
    plt.show()



if __name__ == "__main__":
    singleSpikeTest()
    #saveAndReloadTest()