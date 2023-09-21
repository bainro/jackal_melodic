import numpy as np
import pickle
import os
from placecell import PlaceNetwork
from utils import loadNetwork, saveNetwork

network = PlaceNetwork()
if os.path.exists("chkpt.pkl"):
    data = loadNetwork("chkpt")
    network.loadFromFile(data)

network.numcosts = 5
for cell in network.cells:
    cell.numcosts = 5
    for arr in cell.wgts.values():
        arr.append(1.0)

saveNetwork(network, "chkpt")