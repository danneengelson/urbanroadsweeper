
import numpy as np
import open3d as o3d
from exjobb.Parameters import CELL_SIZE

NO_CELL = -1

class Cell:
    '''Class representing a cell in a Discrete Elevation Model of the environment
    '''
    def __init__(self, elevation, points_idx_in_full_pcd, coverable_points_idx_in_full_pcd):
        '''
        Args:
            elveation: height of the cell, a z-value.
            points_idx_in_full_pcd: indexes of the points in the full point cloud
                                    that are inside this cell
        '''
        self.points_idx_in_full_pcd = points_idx_in_full_pcd
        self.coverable_points_idx_in_full_pcd = coverable_points_idx_in_full_pcd
        self.elevation = elevation
        self.is_traversable = False

class Floor:
    ''' Class representing a floor in a multi-floor environment
    '''
    def __init__(self, name, full_pcd, points_idx=np.array([])):
        '''
        Args:
            name: name of the floor
            full_pcd: Full Point Cloud including other floors
            points_idx: indexes of the points in the full point cloud
                        that are inside this floor
        '''
        self.name = name
        self.points_idx_in_full_pcd = points_idx

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.asarray(full_pcd.points)[points_idx.astype(int)])
        
        bounding_box = self.pcd.get_axis_aligned_bounding_box()
        self.min_x = bounding_box.min_bound[0]
        self.min_y = bounding_box.min_bound[1]
        self.min_z = bounding_box.min_bound[2]
        self.max_x = bounding_box.max_bound[0]
        self.max_y = bounding_box.max_bound[1]
        self.max_z = bounding_box.max_bound[2] 
        self.z_ground = self.min_z
        self.z_ceiling = self.max_z

        self.cells_grid = self.make_2D_grid()

    def make_2D_grid(self):
        ''' Creates a 2D grid of the floor with cells of size CELL_SIZE.
        Returns:
            An empty 2D-numpy array.
        '''
        nbr_of_x = int(np.ceil((self.max_x-self.min_x) / CELL_SIZE)) 
        nbr_of_y = int(np.ceil((self.max_y-self.min_y) / CELL_SIZE)) 
        grid = np.empty((nbr_of_x, nbr_of_y), dtype=object)
        return grid

    def add_cell(self, x_idx, y_idx, elevation, points_idx_in_full_pcd, coverable_points_idx_in_full_pcd):
        ''' Adds a Cell Class instance to the grid. Used by the Elevation
        Detector to assign elevations to cells.
        Args:
            x_idx: index of the cell in x-direction 
            y_idx: index of the cell in y-direction 
            elevation: height of the cell in the point cloud
            points_idx_in_full_pcd: indexes of the points in the full point cloud
                                    that are inside this cell
            points_idx_in_full_pcd: indexes of the points in the full point cloud
                                    that are inside this cell and coverable

        '''
        new_cell = Cell(elevation, points_idx_in_full_pcd, coverable_points_idx_in_full_pcd)
        self.cells_grid[x_idx, y_idx] = new_cell

    def is_valid_cell(self, pos):
        ''' Checks if a position in the grid has an assigned Cell instance.
        Args:
            pos: a tuple representing the position in the 2D grid as (x,y)
        '''
        try:
            cell = self.cells_grid[pos]
            if cell is None:
                return False
                
            return True
        except:
            return False

    def pos_to_position(self, pos):
        x = self.min_x + pos[0]*CELL_SIZE + CELL_SIZE/2
        y = self.min_y + pos[1]*CELL_SIZE + CELL_SIZE/2
        return x, y