import numpy as np
import sys
from pprint import pprint
from run_config import ENVIRONMENT
from exjobb.Environments import PointCloudEnvironment
 
def get_random_point(all_points):
    return all_points[np.random.randint(len(all_points))]

def main():
    pcd_name = sys.argv[1]
    pcd_file_name = ENVIRONMENT[pcd_name]["file"]
    nbr_of_start_points_points = int(sys.argv[2])
            
    # Classify Point Cloud
    environment = PointCloudEnvironment(print, pcd_file_name + ".dictionary", pcd_file_name)
    coverable_points = environment.coverable_pcd.points
    traversable_points = environment.traversable_pcd.points 

    start_points = {}
    for i in np.arange(nbr_of_start_points_points):
        start_points[i] = get_random_point(traversable_points)
        
    
    pprint(start_points)




if __name__ == '__main__':
    main()