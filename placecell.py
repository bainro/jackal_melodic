import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from utils import haversineDistance, saveNetwork, loadNetwork, shortenLine, get_distance

class PlaceCell:
    """
    Class for an individual place cell.
    """
    def __init__(self, ID, x, y, numcosts):
        self.ID = ID
        self.origin = (x, y)
        self.initwgt = 1.0
        self.numcosts = numcosts
        
        #Connections map ID to cell
        self.connections = {}
        self.wgts = {}
        
        #Wavefront vars
        self.v = 0
        self.u = 0
        self.I = 0
        self.delaybuffs = {}
        self.et = 0.0

        self.visitation = 0
    
        
    def reset(self):
        self.v = 0
        self.u = 0
        self.I = 0
        self.et = 0.0
        for key in self.delaybuffs.keys():
            self.delaybuffs[key] = 0
        
    def connect(self, cell):
        """
        Connects cell with input cell. Weight between the two is set to init value.
        """
        self.connections[cell.ID] = cell
        cell.connections[self.ID] = self
        
        self.wgts[cell.ID] = [self.initwgt for i in range(self.numcosts)]
        cell.wgts[self.ID] = [self.initwgt for i in range(self.numcosts)]
        
        #Spikewave delay buff
        self.delaybuffs[cell.ID] = 0
        cell.delaybuffs[self.ID] = 0

class PlaceNetwork:
    """
    Class for a network of place cells. Contains array of cells in the order of initialization (also ID order)
    """
    def __init__(self):
        self.cells = []

        #Maps ID to cartesian points   
        self.points = {}
        
    def isConnected(self, id1, id2):
        if id1 in self.cells[id2].connections.keys() and id2 in self.cells[id1].connections.keys():
            return True
        else:
            return False
        
    def addCell(self, cell):
        """
        Adds cell to network
        """
        self.cells.append(cell)

    def getAvgWgt(self, cell, costmap):
        avg = 0
        for connected in cell.wgts.keys():
            avg += self.cells[connected].wgts[cell.ID][costmap]
        return avg / len(cell.wgts.keys())        

    def plotPath(self, path, costmap=0, image=None, title="Path", diff_map=None):

        if diff_map is not None:
            network = diff_map
        else:
            network = self

        arrow_properties = dict(
            arrowstyle='->, head_length=0.4, head_width=0.4',
            linewidth=2
        )

        wgts = network.normalizeWeights(costmap)

        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_axes([0.05, 0.05, 0.85, 0.85])
        ax.tick_params(left=False, right=False, labelleft=False, labelbottom=False, bottom=False)

        #Get colors
        colors = []

        for cell in self.cells:
            mean = []
            for con in self.cells:
                if (con.ID, cell.ID) in wgts.keys():
                    mean.append(wgts[(con.ID, cell.ID)])
            colors.append(np.mean(mean))

        colors = np.array(colors)
        cmap = plt.get_cmap('inferno')
        norm = Normalize(vmin=1.0, vmax=10.0)
        sm = ScalarMappable(cmap=cmap, norm=norm)
        color_vectors = sm.to_rgba(colors, alpha=None)

        ax_colorbar = fig.add_axes([0.925, 0.1, 0.03, 0.65])
        cbar = plt.colorbar(sm, cax=ax_colorbar)

        cellpairs = []

        for i, cell in enumerate(self.cells):
            if cell.ID in path:
                alph = 1.0
                ms = 15
            else:
                alph = 0.25
                ms = 12

            if colors[i] == 1.0: 
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms, color="black", zorder=2, alpha=alph)
            else:
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms, color=color_vectors[i], zorder=2, alpha=alph)

            for connected_cell in cell.connections.values():
                conncell = (self.points[connected_cell.ID][0], self.points[connected_cell.ID][1])
                cell_short, conn_short = shortenLine((self.points[cell.ID][1], self.points[cell.ID][0]), (conncell[1], conncell[0]), .15)
                cell_arr, conn_arr = shortenLine((self.points[cell.ID][1], self.points[cell.ID][0]), (conncell[1], conncell[0]), .075)
                #print((self.points[cell.ID][0], self.points[cell.ID][1]), (conncell[0], conncell[1]))
                #print(cell_short, conn_short)
                #exit()

                #Darken those in path
                if cell.ID in path and connected_cell.ID in path and path.index(cell.ID) + 1 == path.index(connected_cell.ID):
                    alph = 1.0
                    linewidth = 2.0
                    if wgts[(cell.ID, connected_cell.ID)] == 1.0:
                        color = 'black'
                    else:
                        color = cmap(norm(wgts[(connected_cell.ID, cell.ID)]))
                    
                    arrow_properties['edgecolor'] = color
                    ax.annotate('', xy=(cell_arr[0], cell_arr[1]),xytext=(conn_arr[0], conn_arr[1]), arrowprops=arrow_properties, color=color)
                else:
                    alph = 0.45
                    linewidth = 0.5
        
                #conncell = (connected_cell.origin[0], connected_cell.origin[1])
                #plt.plot([cell.origin[1], conncell[1]], [cell.origin[0], conncell[0]], 'ko-', zorder=0)

                #ax.plot([self.points[cell.ID][1], conncell[1]], [self.points[cell.ID][0], conncell[0]],'k-', zorder=1, linewidth=linewidth, marker='', alpha=alph)
                    ax.plot([cell_short[0], conn_short[0]], [cell_short[1], conn_short[1]],'k-', zorder=1, linewidth=linewidth, marker='', alpha=alph)
                cellpairs.append((min(cell.ID, connected_cell.ID), max(cell.ID, connected_cell.ID)))

        ax.set_title(title, fontsize=20)

        if image is not None:
            background_image = mpimg.imread(image)
            square_x = 300   # Starting x-coordinate of the square
            square_y = 0   # Starting y-coordinate of the square
            square_size = 1400   # Size of the square (adjust as needed)

            # Extract the square portion from the rectangular image
            square_image = background_image[square_y:square_y+square_size, square_x:square_x+square_size]
            
            # Set the extent to fit the square plot while maintaining aspect ratio
            ax.imshow(square_image, extent=[-1, 17, -1, 17], aspect='auto', zorder=0, alpha=0.5)

    def plotCells(self, costmap=0, image=None, title="Cost Map"):
        #fig, ax = plt.subplots(figsize=(12, 12))
        fig = plt.figure(figsize=(12, 12))
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
            colors.append(np.mean(mean))

        ms = 15
        colors = np.array(colors)
        cmap = plt.get_cmap('inferno')
        norm = Normalize(vmin=1.0, vmax=10.0)
        sm = ScalarMappable(cmap=cmap, norm=norm)
        color_vectors = sm.to_rgba(colors, alpha=None)

        ax_colorbar = fig.add_axes([0.925, 0.1, 0.03, 0.65])
        cbar = plt.colorbar(sm, cax=ax_colorbar)

        cellpairs = []

        for i, cell in enumerate(self.cells):
            if colors[i] == 1.0: 
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms, color="black", zorder=2)
            else:
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=ms, color=color_vectors[i], zorder=2)
            #Annotate cell with ID
            #plt.annotate(cell.ID, (cell.origin[1], cell.origin[0]), color='blue', zorder=3, fontsize=8)
            #plt.annotate((self.points[cell.ID][0], self.points[cell.ID][1]), (self.points[cell.ID][1], self.points[cell.ID][0] + 0.15), color='blue', zorder=3, fontsize=8)
            #plt.text(cell.origin[1], cell.origin[0], f'{cell.ID}')
            
            for connected_cell in cell.connections.values():
                if (min(cell.ID, connected_cell.ID), max(cell.ID, connected_cell.ID)) in cellpairs:
                    continue
                #conncell = (connected_cell.origin[0], connected_cell.origin[1])
                #plt.plot([cell.origin[1], conncell[1]], [cell.origin[0], conncell[0]], 'ko-', zorder=0)

                conncell = (self.points[connected_cell.ID][0], self.points[connected_cell.ID][1])
                ax.plot([self.points[cell.ID][1], conncell[1]], [self.points[cell.ID][0], conncell[0]],'ko-', zorder=1, linewidth=0.5)
                cellpairs.append((min(cell.ID, connected_cell.ID), max(cell.ID, connected_cell.ID)))


        ax.set_title(title, fontsize=20)

        if image is not None:
            background_image = mpimg.imread(image)
            square_x = 300   # Starting x-coordinate of the square
            square_y = 0   # Starting y-coordinate of the square
            square_size = 1400   # Size of the square (adjust as needed)

            # Extract the square portion from the rectangular image
            square_image = background_image[square_y:square_y+square_size, square_x:square_x+square_size]
            
            # Set the extent to fit the square plot while maintaining aspect ratio
            ax.imshow(square_image, extent=[-1, 17, -1, 17], aspect='auto', zorder=0, alpha=0.5)


    def printLatLon(self):
        print("latitude, longitude")
        for cell in self.cells:
            print(str(cell.origin[0]) + ", " + str(cell.origin[1]))

    def generateGrid(self, center_lat, center_lon, max_length, exclude_list):
        """
        Generate a grid of latitude and longitude points within a specified maximum length.
        """
        spacing = 2.5  # Spacing between adjacent points
        num_points = int(max_length / spacing) + 1
        
        latitudes = np.linspace(center_lat - max_length / 111000, center_lat + max_length / 111000, num_points)
        longitudes = np.linspace(center_lon - max_length / (111000 * np.cos(np.radians(center_lat))),
                                center_lon + max_length / (111000 * np.cos(np.radians(center_lat))), num_points)

        self.mapsizelat = len(latitudes)
        self.mapsizelon = len(longitudes)

        grid = []
        for i in range(len(latitudes)):
            for j in range(len(longitudes)):
                if (i, j) not in exclude_list:
                    grid.append((latitudes[i], longitudes[j]))
        #grid = [(lat, lon) for lat in latitudes for lon in longitudes]
        return grid
    
    def rotateAboutCenter(self, points, center, rotation_degrees):
        rotation_radians = np.radians(rotation_degrees)
        rotation_matrix = np.array([[np.cos(rotation_radians), -np.sin(rotation_radians)],
                                    [np.sin(rotation_radians), np.cos(rotation_radians)]])
        
        rotated_points = []
        center_lat, center_lon = center
        
        for lat, lon in points:
            lat_rad = np.radians(lat)
            lon_rad = np.radians(lon)
            
            # Translate points to be centered around the origin
            translated_lat_rad = lat_rad - np.radians(center_lat)
            translated_lon_rad = lon_rad - np.radians(center_lon)
            
            # Rotate the translated points
            rotated_lat_lon = np.dot(rotation_matrix, np.array([translated_lat_rad, translated_lon_rad]))
            
            # Translate points back to the original center
            rotated_lat_rad = rotated_lat_lon[0] + np.radians(center_lat)
            rotated_lon_rad = rotated_lat_lon[1] + np.radians(center_lon)
            
            rotated_lat = np.degrees(rotated_lat_rad)
            rotated_lon = np.degrees(rotated_lon_rad)
            rotated_points.append((rotated_lat, rotated_lon))
        
        return rotated_points
    
    def addFromGrid(self, grid):
        """
        Adds cells to network from a grid of latitude and longitude points.
        """
        for i in range(len(grid)):
            self.addCell(PlaceCell(i, grid[i][0], grid[i][1], self.numcosts))

    def initAldritch(self, numcosts=1):
        """
        Creates a map for Aldritch park

        numcosts -> number of costmaps
        """
        self.numcosts = numcosts
        center = [33.646362, -117.843127]

        exclude = [(10, 14), (11, 14), (12, 14), (13, 14), (14, 14), (15, 14), (16, 14),
                   (10, 15), (11, 15), (12, 15), (13, 15), (14, 15), (15, 15), (16, 15),
                   (10, 16), (11, 16), (12, 16), (13, 16), (14, 16), (15, 16), (16, 16),
                   (0, 4), (1, 4), (2, 4), (3, 4), (4, 4),
                   (0, 5), (1, 5), (2, 5), (3, 5), (4, 5),
                   (0, 6), (1, 6), (2, 6), (3, 6), (4, 6),
                   (0, 7), (1, 7), (2, 7), (3, 7), (4, 7),
                   (0, 8), (1, 8), (2, 8), (3, 8), (4, 8)]
        #Reverse because I messed up
        exclude = [(pt[1], pt[0]) for pt in exclude]

        grid = self.generateGrid(center[0], center[1], 40, exclude)
        grid = self.rotateAboutCenter(grid, center, 20)
        self.addFromGrid(grid)

        id = 0
        for i in range(self.mapsizelat):
            for j in range(self.mapsizelon):
                if (i, j) not in exclude:
                    self.points[(i, j)] = id
                    self.points[id] = (i, j)
                    id += 1

    def initConnections(self):
        """
        Connects cells in network
        """
        for cell in self.cells:
            for other_cell in self.cells:
                if cell.ID != other_cell.ID:
                    if self.isConnected(cell.ID, other_cell.ID):
                        continue
                    else:
                        if haversineDistance(cell.origin[0], cell.origin[1], other_cell.origin[0], other_cell.origin[1]) < 7.75:
                            cell.connect(other_cell)


    def normalizeWeights(self, costmaps):
        """
        Returns a dictionary of wgts that are normalized in relation to each costmap provided.
        """

        #New dict maps cell ID pair to weights
        wgt_dict = {}
        for cell in self.cells:
            for cons in cell.wgts.keys():
                wgt_dict[(cell.ID, cons)] = sum([cell.wgts[cons][j] for j in costmaps])
            
        #Normalize dict values between 1-10
        max_wgt = max(wgt_dict.values())
        min_wgt = min(wgt_dict.values())
        if max_wgt != min_wgt:
            for key in wgt_dict.keys():
                wgt_dict[key] = 1 + (wgt_dict[key] - min_wgt) * 9 / (max_wgt - min_wgt)

        return wgt_dict

    def RRTstar(self, startPt, goalPt, costmap=[0], rewire=True):
        """
        Performs RRT* on graph.
        """

        class Node():
            def __init__(self, ID, parent=None, cost=0):
                self.ID = ID
                self.parent = parent
                self.children = set()
                self.cost = cost

            def computeCost(self, cost, wgts):
                if self.parent is not None:
                    self.cost = self.parent.cost + cost
                else:
                    self.cost = 0

                #Propagate downwards
                for child in self.children:
                    child.computeCost(wgts[(self.ID, child.ID)], wgts)

        wgt_dict = self.normalizeWeights(costmap)
        startID = self.points[startPt[0], startPt[1]]
        goalID = self.points[goalPt[0], goalPt[1]]


        choices = set(list(range(len(self.cells))))
        choices.remove(startID)
        startNode = Node(startID)
        nodelist = {startID: startNode}
        reached = False
        while not reached:

            closest = None
            best = None
            while closest is None or best is None:
                randID = np.random.choice(list(choices))
                #Find closest
                closest = None
                closest_dist = None
                for node in nodelist.values():
                    dist = get_distance(self.points[node.ID], self.points[randID])
                    if closest is None:
                        closest = node
                        closest_dist = dist
                    elif dist < closest_dist:
                        closest = node
                        closest_dist = dist
                #Extend in direction of randID
                closest_neigh = None
                closest_neigh_dist = None
                for cons in self.cells[closest.ID].connections.keys():
                    if cons in choices:
                        dist = get_distance(self.points[cons], self.points[randID])
                        if closest_neigh is None:
                            closest_neigh = cons
                            closest_neigh_dist = dist
                        elif dist < closest_neigh_dist:
                            closest_neigh = cons
                            closest_neigh_dist = dist

                best = closest_neigh
            best_cost = wgt_dict[(closest.ID, best)]
            #Search surrounding nodes for better cost
            if rewire:
                for shared in self.cells[closest.ID].connections.keys():
                    if shared in self.cells[closest_neigh].connections.keys() and shared in choices:
                        cost = wgt_dict[(closest.ID, shared)]
                        if cost < best_cost:
                            best = shared
                            best_cost = cost        

            #Closest is a Node, closest_neigh is an ID
            choices.remove(best)
            newnode = Node(best, closest)
            newnode.computeCost(wgt_dict[(closest.ID, best)], wgt_dict)
            nodelist[best] = newnode
            closest.children.add(newnode)

            #Rewiring
            if rewire:
                for con in self.cells[best].connections.keys():
                    if con in nodelist.keys():
                        cost = nodelist[con].cost + wgt_dict[(con, best)]
                        if cost < newnode.cost:
                            newnode.parent.children.remove(newnode)
                            newnode.parent = nodelist[con]
                            newnode.parent.children.add(newnode)
                            newnode.computeCost(wgt_dict[(con, best)], wgt_dict)


            if best == goalID:
                reached = True

        path = [goalID]
        parent = nodelist[goalID].parent
        while parent != None:
            path.append(parent.ID)
            parent = parent.parent
    
        #fig = plt.figure(figsize=(12, 12))
        #ax = fig.add_axes([0.05, 0.05, 0.85, 0.85])
        #ax.tick_params(left=False, right=False, labelleft=False, labelbottom=False, bottom=False)

        #Debug Graph
        # points = [self.points[i] for i in range(len(self.cells))]
        # plt.scatter([p[1] for p in points], [p[0] for p in points], color='black', alpha=0.5, zorder=0)

        # for node in nodelist.values():
        #     for child in node.children:
        #         if node.ID in path and child.ID in path and path.index(node.ID) - 1 == path.index(child.ID):
        #             plt.plot([self.points[node.ID][1], self.points[child.ID][1]], [self.points[node.ID][0], self.points[child.ID][0]], 'ro-', zorder=2)
        #         else:
        #             plt.plot([self.points[node.ID][1], self.points[child.ID][1]], [self.points[node.ID][0], self.points[child.ID][0]], 'ko-', zorder=1)
        # plt.title("RRT* Graph", fontsize=20)
        # plt.savefig("images/rrt_star_graph.png")
        # plt.show()
        # plt.close()
        ############

        return path

    def spikeWave(self, startPt, goalPt, costmap=[0]):
        """
        Performs spikewave propogation on graph.
        """

        wgt_dict = self.normalizeWeights(costmap)

        startID = self.points[startPt[0], startPt[1]]
        goalID = self.points[goalPt[0], goalPt[1]]

        #Reset all neurons
        for cell in self.cells:
            cell.reset()
        
        spike = 1
        refractory = -20

        #Eligitibilty trace time constant
        et_tc = 25.0
        
        #Spike initiated from starting location
        self.cells[startID].v = spike
        
        found_goal = False
        time_steps = 0
        aer = []
        
        while not found_goal:
            time_steps += 1
            
            #Neurons that spiked
            inx = 0
            fid = []
            for cell in self.cells:
                if cell.v >= spike:
                    fid.append(cell.ID)
                    aer.append([time_steps, cell.ID])
                    inx += 1
                    
            #Spiked neurons send spike to connections
            for i in range(inx):
                
                #Setting spiked neurons refractory
                self.cells[fid[i]].u = refractory

                #Eligibility trace
                self.cells[fid[i]].et = 1.0

                #if self.cells[fid[i]].ID == 0:
                #    print("Spike at start location")


                #Set delay buffers of connections as weights of those connections:
                for cons in self.cells[fid[i]].delaybuffs.keys():
                    self.cells[fid[i]].delaybuffs[cons] = round(wgt_dict[(fid[i], cons)])#round(sum([self.cells[fid[i]].wgts[cons][j] for j in costmap]))
                    
                #Check if neuron is goal locations
                if self.cells[fid[i]].ID == goalID:
                    found_goal = True
                    
            if not found_goal:
                
                #Update I first
                for cell in self.cells:
                    cell.I = cell.u
                    
                #Then decrement delay buffers
                for cell in self.cells:
                    for cons in cell.delaybuffs.keys():
                        if cell.delaybuffs[cons] == 1:
                            #If delay over, increase the current of connected cell
                            cell.connections[cons].I += 1
                        
                        cell.delaybuffs[cons] = max(0, cell.delaybuffs[cons] - 1)

                    #Update eligibility trace
                    cell.et -= cell.et / et_tc
                
                for cell in self.cells:
                    cell.v = cell.I
                    cell.u = min(cell.u + 1, 0)
            
        path = self.getPath(aer, startID, goalID)
        return path
    
    def getPath(self, spks, s, e):
        """
        Returns the reverse path using most spiketable
        """
        path = list()
        path.append(e)
        
        for i in range(len(spks)):
            if spks[i][1] == e:
                curr_spike_idx = i
                break
                
        while(path[-1] != s):
            connecting_spike = None
            oldest_idx = -1
            #Get neighboring spike

            connected_spks = []
            connected_spk_times = []

            for i in range(curr_spike_idx - 1, -1, -1):
                if self.isConnected(spks[curr_spike_idx][1], spks[i][1]):
                    connected_spks.append((spks[i][1], i))
                    connected_spk_times.append(spks[i][0])
                    connecting_spike = spks[i]
                    oldest_idx = i
            
            connected_spks = np.array(connected_spks)
            connected_spk_times = np.array(connected_spk_times)
            min_indxs = np.where(connected_spk_times == np.amin(connected_spk_times))[0]

            min_distance = None
            for sp in connected_spks[min_indxs]:
                distance = get_distance(self.points[sp[0]], self.points[spks[curr_spike_idx][1]])
                if min_distance is None:
                    min_distance = distance
                    connecting_spike = spks[sp[1]]
                    oldest_idx = sp[1]
                elif distance < min_distance:
                    min_distance = distance
                    connecting_spike = spks[sp[1]]
                    oldest_idx = sp[1]



            curr_spike_idx = oldest_idx
            path.append(connecting_spike[1])
            
        
        #exit()
        return path
    
    def eProp(self, costs, path):
        """
        Performs eprop

        costs: list of costs for each step in path
        path: list of cells in the path
        """
        lr = 0.5

        #Check there is a valid cost for each costmap at each path
        for cost in costs:
            if len(cost) != self.numcosts:
                print("Invalid cost list")
                return

        #Reverse path
        path = path[::-1]
        #print(path)
        for i in range(len(path) - 1):

            updated_ids = []
            for cell in self.cells:
                if path[i+1] in cell.wgts.keys():
                    updated_ids.append(cell.ID)

            for j in range(self.numcosts):
                if costs[i][j] != -1:
                    #print(f"Loss {path[i]} to {path[i + 1]}: ", loss)
                    for id in updated_ids:
                        loss = costs[i][j] - self.cells[id].wgts[path[i + 1]][j]
                        self.cells[id].wgts[path[i+1]][j] += lr * loss * self.cells[id].et

    def loadFromFile(self, file):

        self.mapsizelat = file[0][0]
        self.mapsizelon = file[0][1]

        self.numcosts = file[1]

        self.points = file[2]

        for cell in file[3]:
            newcell = PlaceCell(cell[0], cell[1][0], cell[1][1], self.numcosts)
            newcell.wgts = cell[2]
            newcell.delaybuffs = cell[3]
            newcell.visitation = cell[4]
            self.addCell(newcell)


        for cell in self.cells:
            for cons in cell.wgts.keys():
                cell.connections[cons] = self.cells[cons]
                self.cells[cons].connections[cell.ID] = cell


if __name__ == "__main__":
    network = PlaceNetwork()
    data = loadNetwork("fixed_wgts")
    network.loadFromFile(data)

    naive_network = PlaceNetwork()
    naive_network.initAldritch(numcosts=6)
    naive_network.initConnections()
    #(15, 1) to (9, 11) for obstacles

    rrt_p = network.RRTstar((15, 1), (9, 11), costmap=[0, 1, 4, 5])
    sw_p = network.spikeWave((15, 1), (9, 11), costmap=[0, 1, 4, 5])


    network.plotPath(rrt_p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="RRT* Path (15, 1) to (9, 11)")
    network.plotPath(sw_p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="SWP Path (15, 1) to (9, 11)")
    #plt.savefig("images/1_rrt_star_combined.png")
    plt.show()

    '''
    p = network.spikeWave((15, 1), (9, 11), costmap=[0, 1, 4, 5])
    network.plotPath(p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Trained Path (15, 1) to (9, 11)")
    plt.savefig("images/1_trained_combined.png")
    #plt.show()
    plt.close()

    p = naive_network.spikeWave((15, 1), (9, 11), costmap=[1])
    naive_network.plotPath(p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Naive Path (15, 1) to (9, 11)", diff_map=network)
    plt.savefig("images/1_naive_obs.png")
    #plt.show()
    plt.close()

    p = network.spikeWave((16, 6), (6, 6), costmap=[0, 1, 4, 5])
    network.plotPath(p, costmap=[1], image="images/map/mapraw.jpg", title="Trained Path (16, 6) to (6, 6)")
    plt.savefig("images/2_trained_combined.png")
    #plt.show()
    plt.close()

    p = naive_network.spikeWave((16, 6), (6, 6), costmap=[1])
    naive_network.plotPath(p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Naive Path (16, 6) to (6, 6)", diff_map=network)
    plt.savefig("images/2_naive_obs.png")
    #plt.show()
    plt.close()

    p = network.spikeWave((0, 13), (5, 13), costmap=[0, 1, 4, 5])
    network.plotPath(p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Trained Path (0, 13) to (5, 13)")
    plt.savefig("images/3_trained_combined.png")
    #plt.show()
    plt.close()

    p = naive_network.spikeWave((0, 13), (5, 13), costmap=[4])
    naive_network.plotPath(p, costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Naive Path (0, 13) to (5, 13)", diff_map=network)
    plt.savefig("images/3_naive_slope.png")
    #plt.show()
    plt.close()

    network.plotCells(costmap=[0], image="images/map/mapraw.jpg", title="Current Cost Map")
    plt.savefig("images/current_cost_map.png")
    #plt.show()
    plt.close()

    network.plotCells(costmap=[1], image="images/map/mapraw.jpg", title="Obstacle Cost Map")
    plt.savefig("images/obstacle_cost_map.png")
    #plt.show()
    plt.close()

    network.plotCells(costmap=[4], image="images/map/mapraw.jpg", title="Slope Cost Map")
    plt.savefig("images/slope_cost_map.png")
    #plt.show()
    plt.close()

    network.plotCells(costmap=[5], image="images/map/mapraw.jpg", title="Blocked Cost Map")
    plt.savefig("images/block_cost_map.png")
    #plt.show()
    plt.close()

    network.plotCells(costmap=[0, 1, 4, 5], image="images/map/mapraw.jpg", title="Combined Cost Map")
    plt.savefig("images/combined_cost_map.png")
    #plt.show()
    plt.close()

    count_one = 0
    count_two = 0
    count_three = 0
    for cell in network.cells:
        if cell.visitation > 0:
            count_one += 1
        if cell.visitation > 1:
            count_two += 1
        if cell.visitation > 2:
            count_three += 1

    print("Cells visited once: ", count_one/len(network.cells))
    print("Cells visited twice: ", count_two/len(network.cells))
    print("Cells visited three times: ", count_three/len(network.cells))
    '''
    
