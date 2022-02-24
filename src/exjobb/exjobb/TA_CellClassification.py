
import numpy as np
import timeit
import operator
from exjobb.PointCloud import PointCloud
from exjobb.Parameters import MAX_STEP_HEIGHT, ROBOT_SIZE, CELL_SIZE
# GROUND_OFFSET

class CellClassification:
    ''' Class for detecting obstacles and unaccessible areas from the DEM, Discrete
    Elevation Model to find coverable areas.
    '''
    def __init__(self, print, param):
        ''' 
        Args:
            print: function for printing messages
        '''
        self.print = print
        self.param = param

    def get_coverable_points_idx(self, full_pcd, floor):
        ''' Finds points that are accessable. No robot size properties are taken into concern.
        Args:
            full_pcd: Point Cloud of the environment including other floors.
            floor: Floor instance with the DEM.
        '''
        start = timeit.default_timer()
        ground_cells_poses = self.get_ground_cells_poses(floor)

        accessible_cells_poses = self.get_accessible_cells_poses(ground_cells_poses, floor)

        uncoverable_border_points = np.empty((0,3))
        coverable_points_idx = np.array([], int)

        for i, pos in enumerate(accessible_cells_poses):
            #self.print("Working on " + str(i) + " out of " + str(len(accessible_cells_poses)) + str("\r")) 
            cell = floor.cells_grid[pos]
            coverable_points_idx = np.append(coverable_points_idx, cell.coverable_points_idx_in_full_pcd )
            new_uncoverable_border_points = self.get_uncoverable_border_points(pos, accessible_cells_poses, floor)
            uncoverable_border_points = np.append(uncoverable_border_points, new_uncoverable_border_points, axis=0)


        stats = self.print_result(start, coverable_points_idx, accessible_cells_poses)

        return coverable_points_idx, uncoverable_border_points, stats

    
    def get_accessible_cells_poses(self, ground_cells_poses, floor):
        ''' Using breadth first to find all accessable poses. Uses ground cells as
        starting points, which creates mulitple islands of connected cells.
        The cells of the island with most cells are chosen as accessable.
        Args:
            ground_cells_poses: positions of all ground level cells
            floor: Floor instance with the DEM.
        '''
        
        all_islands = []
        ground_cells_queue = [tuple(pair) for pair in ground_cells_poses]

        while ground_cells_queue:
            start_cell = ground_cells_queue.pop()
            visited_cells = [start_cell]
            coverable_cells_queue = [start_cell]
            while coverable_cells_queue:
                current_pos = coverable_cells_queue.pop()
                for neigbour_pos in self.get_neighbours_poses(current_pos):

                    if not floor.is_valid_cell(neigbour_pos):
                        continue   

                    if neigbour_pos in visited_cells:
                        continue 
                    
                    if not self.is_valid_step(current_pos, neigbour_pos, floor):
                        continue                    

                    if neigbour_pos in ground_cells_queue:
                        ground_cells_queue.remove(neigbour_pos)

                    visited_cells.append(neigbour_pos)
                    coverable_cells_queue.append(neigbour_pos)
                
            all_islands.append(visited_cells)

        self.nbr_inaccessible_cells = 0
        for island in all_islands:
            self.nbr_inaccessible_cells += len(island)

        

        cells_in_biggest_island = max(all_islands, key=len)
        self.nbr_inaccessible_cells -= len(cells_in_biggest_island)
        
        for cell in cells_in_biggest_island:
            floor.cells_grid[cell].is_traversable = True

        if self.param["GET_SECOND_BIGGEST_ISLAND"]:
            #Ugly hack to get the second biggest island for second floor (used in bridge)
            if floor.name == "Floor 2":
                self.nbr_inaccessible_cells += len(cells_in_biggest_island)
                all_islands.remove(cells_in_biggest_island)
                cells_in_biggest_island = max(all_islands, key=len)
                self.nbr_inaccessible_cells -= len(cells_in_biggest_island)


        return cells_in_biggest_island

    def get_uncoverable_border_points(self, pos, accessible_cells_poses, floor):
        cell = floor.cells_grid[pos]
        elevation = cell.elevation 
        uncoverable_border_points = np.empty((0,3))

        for neighbour in self.get_neighbours_poses(pos):
                
                if neighbour in accessible_cells_poses:
                    continue

                x, y = floor.pos_to_position(neighbour)
                new_point = np.array([x, y, elevation])
                uncoverable_border_points = np.append(uncoverable_border_points, [new_point], axis=0)

        return uncoverable_border_points

    def get_neighbours_poses(self, cell):
        ''' Returns the position of the north, south, west an east neighbour of a given cell.
        Args:
            cell: tuple with an arbitary position
        '''

        directions = [(1,0), (0,1), (-1, 0), (0, -1)]
        neighbours = []
        for dir in directions:
            neighbour = tuple(map(operator.add, cell, dir))
            neighbours.append(neighbour)

        return neighbours
    
    def is_valid_step(self, from_pos, to_pos, floor):
        ''' Checks whether a step from one cell to another is possible by
        looking at the height difference between the cells.
        Args:
            from_pos: tuple (x,y) representing the position of the start cell
            to_pos: tuple (x,y) representing the position of the goal cell
            floor: Floor instance with the DEM.
        '''
        ground_pos_elevation = floor.cells_grid[from_pos].elevation
        neigbour_pos_elevation = floor.cells_grid[to_pos].elevation

        return abs(neigbour_pos_elevation - ground_pos_elevation) <= MAX_STEP_HEIGHT

    def get_ground_cells_poses(self, floor):
        ''' Finds all ground level cells in the floor.
        Args:
            floor: Floor instance with the DEM. 
        Returns:
            A list of ground cell positions in a tuple format (x,y)
        '''

        def is_ground_level_cell(cell):
            if cell is None:
                return False
            return cell.elevation < floor.z_ground + self.param["GROUND_OFFSET"] - 0.05
        
        x_poses, y_poses = np.where( np.vectorize(is_ground_level_cell)(floor.cells_grid))
        ground_level_cells = np.vstack((x_poses, y_poses)).T

        return ground_level_cells.tolist()

    def delete_values(self, array, values):
        ''' Removes specific values from an array
        Args:
            array: NumPy array to remove values from
            values: NumPy array with values that should be removed.
        '''
        return array[ np.isin(array, values, assume_unique=True, invert=True) ]
 
    def print_result(self, start, coverable_points, coverable_cells):
        ''' Prints result data of the floor segmentation.
        '''
        end = timeit.default_timer()
        self.print("-"*20)
        self.print("CELL CLASSIFICAION")
        self.print("Computational time: " + str(round(end - start, 1)) + " sec")
        self.print("Number of COVERABLE cells: " + str(len(coverable_cells)))
        self.print("Number of INACCESSIBLE cells: " + str(self.nbr_inaccessible_cells))
        stats = {
            "Computational time": round(end - start, 1),
            "Number of COVERABLE cells": len(coverable_cells),
            "Number of INACCESSIBLE cells": self.nbr_inaccessible_cells
        }
        return stats
