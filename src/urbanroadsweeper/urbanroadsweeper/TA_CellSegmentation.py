import numpy as np
import open3d as o3d
import timeit

from urbanroadsweeper.Parameters import CELL_SIZE, Z_RESOLUTION, MIN_FLOOR_HEIGHT, MAX_STEP_HEIGHT, MIN_POINTS_IN_CELL

class CellSegmentation:
    ''' Class for creating a Discrete Elevation Model of the Point Cloud.
    '''

    def __init__(self, print, param):
        ''' 
        Args:
            print: function for printing messages
        '''
        self.print = print
        self.param = param

    def find_elevation(self, full_pcd, floor):
        ''' Creating a Discrete Elevation Model of the Point Cloud by 
        assigning elevation heights to every valid cell in a 2D grid
        of the given floor. 
        Args:
            full_pcd: Point Cloud of the environment including other floors
            floor: Floor Class object of an arbitary floor in the environment.
        '''
        start = timeit.default_timer()

        nbr_of_cells = floor.cells_grid.shape[0]*floor.cells_grid.shape[1]
        nbr_of_valid_cells = 0
        kdtree = o3d.geometry.KDTreeFlann(full_pcd)
        
        points_in_pcd = np.asarray(full_pcd.points)
        z_values = points_in_pcd[:,2]
        points_idx_in_z_range = np.where( np.logical_and(z_values >= floor.z_ground, z_values < floor.z_ceiling ))[0]



        for x_idx in range(floor.cells_grid.shape[0]):
            #self.print(str(x_idx) + "/" + str(floor.cells_grid.shape[0]))
            x = x_idx * CELL_SIZE + floor.min_x
            x_values = points_in_pcd[points_idx_in_z_range, 0]
            points_idx_in_x_range_of_z_values =  np.where( np.logical_and(x_values >= x, x_values < x + CELL_SIZE ))[0]
            points_idx_in_x_and_z_range = points_idx_in_z_range[points_idx_in_x_range_of_z_values]

            for y_idx in range(floor.cells_grid.shape[1]):
            
                #x = x_idx * CELL_SIZE + floor.min_x
                y = y_idx * CELL_SIZE + floor.min_y  

                #if self.approximate_number_of_points_in_cell_is_too_low(kdtree, x, y, floor):
                #    continue


                y_values = points_in_pcd[points_idx_in_x_and_z_range, 1]
                points_idx_in_y_range_of_z_values =  np.where( np.logical_and(y_values >= y, y_values < y + CELL_SIZE ))[0]
                points_idx_in_cell = points_idx_in_x_and_z_range[points_idx_in_y_range_of_z_values]


                #cell_bounding_box = o3d.geometry.AxisAlignedBoundingBox(
                #        [x, y, floor.z_ground],
                #        [x + CELL_SIZE, y + CELL_SIZE, floor.z_ceiling]
                #    )
                #points_idx_in_cell = cell_bounding_box.get_point_indices_within_bounding_box(points_in_pcd)

                #points_in_cell = np.asarray(points_in_pcd)[points_idx_in_cell]
                points_in_cell = points_in_pcd[points_idx_in_cell]

                if len(points_in_cell) < self.param["MIN_POINTS_IN_CELL"]:
                    continue

                
                z_values_of_points_in_cell = np.append(points_in_cell[:,2], floor.z_ceiling)
                elevation = self.get_elevation_of_cell(z_values_of_points_in_cell)

                coverable_points_idx_in_cell = self.get_coverable_points_idx_in_cell(points_in_cell, points_idx_in_cell, elevation)
                

                if len(coverable_points_idx_in_cell) < self.param["MIN_POINTS_IN_CELL"]:
                    continue
                
                nbr_of_valid_cells += 1

                floor.add_cell(x_idx, y_idx, elevation, points_idx_in_cell, coverable_points_idx_in_cell)

        stats = self.print_result(start, nbr_of_valid_cells, nbr_of_cells)  

        return stats


    def get_coverable_points_idx_in_cell(self, points, points_idx, elevation):  
        ''' Finds points in the cell that are coverable by taking the points close
        to the elvation level of the cell.
        Args:
            points: Nx3 array with all points in the cell
            points_idx: indicies of these points in the full point cloud.        
        '''
        z_values = points[:,2]        
        coverable = np.where(  abs(z_values - elevation) < MAX_STEP_HEIGHT  )[0]
        coverable = np.array(coverable, int)

        return np.take(points_idx, coverable)

    def get_elevation_of_cell(self, z_values):    
        ''' Starting from the lowest point, it looks for an empty space in the cell
        with a height of at least the MIN_FLOOR_HEIGHT, where there are no points.
        Args:
            z_values: List of the z-values of all points in the cell
        '''

        sorted_z_values = np.sort(z_values)

        for idx, z in enumerate(sorted_z_values[1:]):
            prev_z = sorted_z_values[idx]
            
            if abs(z - prev_z) > MIN_FLOOR_HEIGHT:
                break
                
        return prev_z

    def approximate_number_of_points_in_cell_is_too_low(self, kdtree, x, y, floor):
        ''' Quick approximate check to see if a cell is valid. On different heights,
        it checks whether the number of points in a radius is bigger than the minimum
        amount of MIN_POINTS_IN_CELL. The purpose is to lower the computational time.
        Args:
            kdtree: a Open3D KDTreeFlann representation of the point lcoud
            x: x-value of the cell
            y: y-value of the cell
            floor: The Floor where the cell is included. 
        
        '''
        
        x_center = x + CELL_SIZE/2
        y_center = y + CELL_SIZE/2
        z_values = np.arange( floor.z_ground, floor.z_ceiling, Z_RESOLUTION)
        
        radius = CELL_SIZE / np.sqrt(2)
        for z in z_values:
            center_point = np.asarray([x_center, y_center, z])
            [k, idx, _] = kdtree.search_radius_vector_3d(center_point, radius)
            
            if len(idx) > self.param["MIN_POINTS_IN_CELL"]:
                return False
        
        return True


    def print_result(self, start, nbr_of_valid_cells, nbr_of_cells):
        ''' Prints result data of the floor segmentation.
        '''
        end = timeit.default_timer()
        self.print("-"*20)
        self.print("CELL SEGMENTATION")
        self.print("Computational time: " + str(round(end - start, 1)) + " sec")
        self.print("Number of INVALID cells: " + str(nbr_of_cells - nbr_of_valid_cells))            
        self.print("TOTAL Number of cells: " + str(nbr_of_cells))   
        return {
            "Computational time": round(end - start, 1),
            "Number of INVALID cells": nbr_of_cells - nbr_of_valid_cells,
            "TOTAL Number of cells": nbr_of_cells
        }