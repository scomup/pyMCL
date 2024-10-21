import yaml
from PIL import Image, ImageOps
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from mathR.utilities.math_tools import *


def mark_likelihood(array, max_distance):
    # Create a binary mask where 1 represents occupied pixels
    occupied_mask = (array == 1)
    
    # Compute the distance transform (distance to the nearest occupied pixel)
    distance_map = distance_transform_edt(~occupied_mask)
    
    # Mark pixels within the specified distance (max_distance)
    cost_map = np.where(distance_map <= max_distance, distance_map, max_distance)
    cost_map = (max_distance - cost_map) / max_distance
    return cost_map

class GridMap:
    def __init__(self, yaml_file):
        # Load YAML file
        with open(yaml_file, 'r') as file:
            map_metadata = yaml.safe_load(file)
        yaml_file = Path(yaml_file)

        image_file = Path(yaml_file.parent/map_metadata['image'])
        im = Image.open(image_file)
        im_flip = ImageOps.flip(im)
        map_array = np.array(im_flip)

        self.occupancy_grid = np.zeros([map_array.shape[0], map_array.shape[1]])
        occupancy_px = map_array == [0]
        self.occupancy_grid[occupancy_px] = 1
        self.occupancy_likelihood = mark_likelihood(self.occupancy_grid, 10)
        self.resolution = map_metadata['resolution'] 
        self.origin = np.array(map_metadata['origin'][:2])
        self.theta = map_metadata['origin'][2]
        self.width = map_array.shape[1]
        self.height = map_array.shape[0]

    def world_to_grid(self, cloud):
        # Apply inverse of origin and resolution to convert to grid index
        grid_coords = (cloud - self.origin) / self.resolution
        grid_coords = grid_coords.astype(np.int32)
        y_check = np.logical_and(grid_coords[:, 1] >= 0 , grid_coords[:, 1] < self.height ) 
        x_cehck = np.logical_and(grid_coords[:, 0] >= 0 , grid_coords[:, 0] < self.width ) 
        grid_coords = grid_coords[np.logical_and(x_cehck, y_check)]
        return grid_coords
    
    def get_weight(self, cloud):
        grid_coords = self.world_to_grid(cloud)
        return np.sum(self.occupancy_likelihood[grid_coords[:,1], grid_coords[:,0]])
    

if __name__ == "__main__":
    map_fn = "/home/liu/workspace/pyMCL/data/map.yaml"
    map = GridMap(map_fn)
    _, ax = plt.subplots()
    ax.imshow(map.occupancy_likelihood, origin='lower')
    plt.show()
