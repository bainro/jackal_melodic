import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from utils import haversineDistance, saveNetwork, loadNetwork

class PlaceCell:
    """
    Class for an individual place cell.
    """
    def __init__(self, ID, x, y, numcosts):
        self.ID = ID
        self.origin = (x, y)
        self.initwgt = 5.0
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

    def plotCells(self, costmap=0, image=None, title="Cost Map"):
        #fig, ax = plt.subplots(figsize=(12, 12))
        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_axes([0.05, 0.05, 0.85, 0.85])
        ax.tick_params(left=False, right=False, labelleft=False, labelbottom=False, bottom=False)

        #Get colors
        colors = []
        for cell in self.cells:
            colors.append(self.getAvgWgt(cell, costmap))
        colors = np.array(colors)
        cmap = plt.get_cmap('viridis')
        norm = Normalize(vmin=colors.min(), vmax=colors.max())
        sm = ScalarMappable(cmap=cmap, norm=norm)
        color_vectors = sm.to_rgba(colors, alpha=None)

        ax_colorbar = fig.add_axes([0.925, 0.1, 0.03, 0.65])
        cbar = plt.colorbar(sm, cax=ax_colorbar)


        for i, cell in enumerate(self.cells):
            if colors[i] == 1.0: 
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=10, color="black", zorder=2)
            else:
                ax.plot(self.points[cell.ID][1], self.points[cell.ID][0], marker='o', ms=10, color=color_vectors[i], zorder=2)
            #Annotate cell with ID
            #plt.annotate(cell.ID, (cell.origin[1], cell.origin[0]), color='blue', zorder=3, fontsize=8)
            #plt.annotate((self.points[cell.ID][0], self.points[cell.ID][1]), (self.points[cell.ID][1], self.points[cell.ID][0] + 0.15), color='blue', zorder=3, fontsize=8)
            #plt.text(cell.origin[1], cell.origin[0], f'{cell.ID}')
            
            for connected_cell in cell.connections.values():
                #conncell = (connected_cell.origin[0], connected_cell.origin[1])
                #plt.plot([cell.origin[1], conncell[1]], [cell.origin[0], conncell[0]], 'ko-', zorder=0)

                conncell = (self.points[connected_cell.ID][0], self.points[connected_cell.ID][1])
                ax.plot([self.points[cell.ID][1], conncell[1]], [self.points[cell.ID][0], conncell[0]],'ko-', zorder=1, linewidth=0.5)

        ax.set_title(title, fontsize=20)

        if image is not None:
            background_image = mpimg.imread(image)
            square_x = 300   # Starting x-coordinate of the square
            square_y = 0   # Starting y-coordinate of the square
            square_size = 1400   # Size of the square (adjust as needed)

            # Extract the square portion from the rectangular image
            square_image = background_image[square_y:square_y+square_size, square_x:square_x+square_size]
            
            # Set the extent to fit the square plot while maintaining aspect ratio
            ax.imshow(square_image, extent=[-1, 17, -1, 17], aspect='auto', zorder=0)


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

    def spikeWave(self, startPt, goalPt, costmap=[0]):
        """
        Performs spikewave propogation on graph.
        """

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
                    self.cells[fid[i]].delaybuffs[cons] = round(sum([self.cells[fid[i]].wgts[cons][j] for j in costmap]))
                    
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
            for i in range(curr_spike_idx - 1, -1, -1):
                if self.isConnected(spks[curr_spike_idx][1], spks[i][1]):
                    connecting_spike = spks[i]
                    oldest_idx = i
            
            curr_spike_idx = oldest_idx
            path.append(connecting_spike[1])
            
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
            for j in range(self.numcosts):
                if costs[i][j] != -1:
                    loss = costs[i][j] - self.cells[path[i]].wgts[path[i + 1]][j]
                    #print(f"Loss {path[i]} to {path[i + 1]}: ", loss)
                    self.cells[path[i]].wgts[path[i+1]][j] += lr * loss * self.cells[path[i]].et

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
    data = loadNetwork("chkpt")
    network.loadFromFile(data)

    network.plotCells(costmap=0, image="images/map/mapraw.jpg", title="Current Cost Map")
    plt.savefig("images/current_cost_map.png")
    plt.show()
    plt.close()

    network.plotCells(costmap=1, image="images/map/mapraw.jpg", title="Obstacle Cost Map")
    plt.savefig("images/obstacle_cost_map.png")
    plt.show()
    plt.close()

    network.plotCells(costmap=2, image="images/map/mapraw.jpg", title="GPS Cost Map")
    plt.savefig("images/gps_cost_map.png")
    plt.show()
    plt.close()

    network.plotCells(costmap=3, image="images/map/mapraw.jpg", title="WIFI Cost Map")
    plt.savefig("images/wifi_cost_map.png")
    plt.show()
    plt.close()
    

    #network.printLatLon()
