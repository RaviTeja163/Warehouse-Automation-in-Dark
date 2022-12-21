# Main and helper function

from PIL import Image
import numpy as np
from PRM import PRM

import matplotlib.pyplot as plt


def load_map(file_path):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    
    # Rescale the image
    img = img.resize((250, 200), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')

    # Get bianry image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    shelves = [[(37.5,37.5),(37.5,62.5),(37.5,87.5),(37.5,137.5),(37.5,162.5),(37.5,187.5)],\
        [(62.5,37.5),(62.5,62.5),(62.5,87.5),(62.5,137.5),(62.5,162.5),(62.5,187.5)],\
        [(112.5,37.5),(112.5,62.5),(112.5,87.5),(112.5,137.5),(112.5,162.5),(112.5,187.5)],\
        [(137.5,37.5),(137.5,62.5),(137.5,87.5),(137.5,137.5),(137.5,162.5),(137.5,187.5)]]

    map_array = load_map("warehouse.PNG")

    # Planning class
    PRM_planner = PRM(map_array)

    # Search with PRM
    PRM_planner.sample()

    ## Define the Start and Goal Positions here
    goal1_shelf_row = 2
    goal1_shelf_col = 2
    start1 = (87.5, 225)
    
    goal2_shelf_row = 1
    goal2_shelf_col = 6
    start2 = (187.5, 112.5)

    # Find the path
    goal1  = shelves[goal1_shelf_row-1][goal1_shelf_col-1]
    PRM_planner.search(start1, goal1)
    goal2  = shelves[goal2_shelf_row-1][goal2_shelf_col-1]
    PRM_planner.search(start2, goal2)
