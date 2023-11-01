from placecell import PlaceNetwork
from utils import loadNetwork
import numpy as np

trained_network = PlaceNetwork()
data = loadNetwork("wps/wp_350")
trained_network.loadFromFile(data)

cms = [0]#, 1, 4, 5]

for cm in cms:
    for cell in trained_network.cells:

        wgt_vals = {}

        for conn in trained_network.cells:
            if cell.ID in conn.wgts.keys():
                wgt_vals[conn.ID] = conn.wgts[cell.ID][cm]

        #Value of all weights going into cell
        vals = np.array(list(wgt_vals.values()))

        #Count number of values greater than 1
        count = np.sum(vals > 1)

        if count == 1:
            maxval = np.max(vals)
            print("ONE CASE Cell: ", cell.ID)
            print("Setting all 1s to ", maxval)
            for key in wgt_vals.keys():
                if wgt_vals[key] == 1:
                    trained_network.cells[key].wgts[cell.ID][cm] = maxval


        elif count > 1:
            meanval = np.mean(vals[vals > 1])
            print("Multiple CASE Cell: ", cell.ID)
            print("Setting all 1s to ", meanval)
            for key in wgt_vals.keys():
                if wgt_vals[key] == 1:
                    trained_network.cells[key].wgts[cell.ID][cm] = meanval

        