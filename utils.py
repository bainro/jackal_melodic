import numpy as np
import pickle

def haversineDistance(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two points on the Earth's surface
    given their latitude and longitude in decimal degrees.
    
    Parameters:
        lat1, lon1: Latitude and longitude of point 1 in decimal degrees.
        lat2, lon2: Latitude and longitude of point 2 in decimal degrees.
    
    Returns:
        Distance between the two points in meters.
    """
    R = 6371000  # Radius of the Earth in meters

    lat1_rad = np.radians(lat1)
    lon1_rad = np.radians(lon1)
    lat2_rad = np.radians(lat2)
    lon2_rad = np.radians(lon2)

    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    a = np.sin(delta_lat / 2) ** 2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(delta_lon / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = R * c
    
    return distance

def saveNetwork(network, title="network"):
    cells = [[cell.ID, cell.origin, cell.wgts, cell.delaybuffs, cell.visitation] for cell in network.cells]
    points = network.points
    numcosts = network.numcosts
    mapsizes = [network.mapsizelat, network.mapsizelon]
    with open(f"{title}.pkl", "wb") as f:
        pickle.dump([mapsizes, numcosts, points, cells], f)

def loadNetwork(title="network"):
    with open(f"{title}.pkl", "rb") as f:
        file = pickle.load(f)
    
    return file

def levy_flight(dims, step_size):
    step = []
    lf = []

    beta = 1.5
    gamma1 = 1.3293  # equals the gamma function with beta + 1
    gamma2 = 0.9064  # equals the gamma function with (beta+1)/2
    sigma = (gamma1*np.sin(np.pi * beta/2) / (gamma2*beta*2**((beta - 1) / 2)))**(1 / beta)
    u = np.random.normal(0, sigma**2, dims)
    v = np.random.normal(0, 1, dims)
    for i in range(dims):
        step.append(u[i]/abs(v[i])**(1/beta))
        lf.append(step_size*step[i])

    return lf

def get_distance(x, y):
    return np.sqrt(pow(x[0] - y[0], 2.0) + pow(x[1] - y[1], 2.0))
