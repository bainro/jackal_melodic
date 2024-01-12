import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from utils import get_distance
from placecell import PlaceNetwork

SHORT_PATHS = 10
MED_PATHS = 10
LONG_PATHS = 10

def getNewPaths(wps, used, num, min_distance, max_distance):
    paths = []
    for i in range(num):
        start = wps[np.random.randint(0, len(wps))]
        while start in used:
            start = wps[np.random.randint(0, len(wps))]
        used.append(start)
        end = wps[np.random.randint(0, len(wps))]
        while get_distance(start, end) <= min_distance or get_distance(start, end) > max_distance or end in used:
            end = wps[np.random.randint(0, len(wps))]
        used.append(end)
        paths.append((start, end))

    return paths


# Generate paths
if __name__ == '__main__':
    print('Generating paths...')
    network = PlaceNetwork()
    network.initAldritch(numcosts=6)
    network.initConnections()

    wps = []
    for cell in network.cells:
        wps.append((network.points[cell.ID][0], network.points[cell.ID][1]))

    used = []
    short_paths = getNewPaths(wps, used, SHORT_PATHS, 1, 4)
    med_paths = getNewPaths(wps, used, MED_PATHS, 4, 8)
    long_paths = getNewPaths(wps, used, LONG_PATHS, 8, 12)
    all_paths = short_paths + med_paths + long_paths
    


    # convert all_paths to string and write to file
    all_paths_str = []
    for path in all_paths:
        all_paths_str.append("(" + str(path[0][0]) + ',' + str(path[0][1]) + ') (' + str(path[1][0]) + ',' + str(path[1][1]) + ")")
    with open('paths.txt', 'w') as f:
        f.write('\n'.join(all_paths_str))
    print('Done.')