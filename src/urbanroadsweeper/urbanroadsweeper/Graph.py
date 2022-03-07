import pickle
import timeit
import numpy as np 
from urbanroadsweeper.Parameters import GRAPH_DIAG_STEP_SIZE


NORTH = 0
NORTHEAST = 1
EAST = 2
SOUTHEAST = 3
SOUTH = 4
SOUTHWEST = 5
WEST = 6
NORTHWEST = 7

class TraversableGraph():
    def __init__(self, motion_planner, traversable_pcd, start_point) -> None:
        self.pcd = traversable_pcd
        self.motion_planner = motion_planner
        self.step_size = GRAPH_DIAG_STEP_SIZE
        self.breadth_first_search(start_point)
        #with open('cached_breadth_first_search.dictionary', 'wb') as cached_pcd_file:
        #    cache_data = {
        #        "node_points": self.points, 
        #        "neighbours": self.neighbours,
        #        }
        #    pickle.dump(cache_data, cached_pcd_file)


    def astar(self, from_node, to_node):
        start_point = self.points[from_node]
        to_point = self.points[to_node]
        node_path = np.array([]).astype(int)
        path_to_starting_point = self.motion_planner.Astar(start_point, to_point)
        for point in path_to_starting_point:
            node = self.get_node(point)
            if node is not False:
                node_path = np.append(node_path, node)
            

        return np.unique(node_path)


    def breadth_first_search(self, start_pos):
        
        start_node = 0
        self.points = np.array([start_pos])
        self.neighbours = {}
        queue = np.array([0])
        while len(queue):
            #self.print(len(self.points))
            #self.print(len(self.neighbours))
            curr_node, queue = queue[0], queue[1:]
            curr_point = self.points[curr_node]
            neighbours = {}

            for direction, neighbour_point in enumerate(self.get_cell_neighbours(curr_point)):

                if not self.motion_planner.is_valid_step(curr_point, neighbour_point):
                    continue
                
                neighbour_node = self.get_node(neighbour_point)

                if neighbour_node is False:
                    neighbour_node = len(self.points)
                    self.points = np.append(self.points, [neighbour_point], axis=0)
                    queue = np.append(queue, neighbour_node)
                elif neighbour_node == curr_node:
                    continue

                neighbours[direction] = neighbour_node
            
            self.neighbours[curr_node] = neighbours

    def get_cell_neighbours(self, current_position):
        """Finds all neighbours of a given position. 

        Args:
            current_position: A [x,y,z] np.array of the start position 
            step_size: The approximate distance to the neighbours
            angle_offset: Optional. The yaw angle of the robot.

        Returns:
            All 8 neighbours of the given position in following order:
            right, forwardright, forward, forwardleft, left, backleft, back, backright
        """
        directions = []
        for direction_idx in range(8):
            angle = direction_idx/8*np.pi*2 + np.pi/2
            diagonal_compensation = 1
            if direction_idx % 2 == 0:
                diagonal_compensation = np.sqrt(0.5)
            x = current_position[0] + np.cos(angle) * self.step_size * diagonal_compensation
            y = current_position[1] + np.sin(angle) * self.step_size * diagonal_compensation
            z = current_position[2]
            pos = np.array([x, y, z])

            directions.append(self.pcd.find_k_nearest(pos, 1)[0])

        return directions

    def get_node(self, point):
        ''' Finds the node (index in self.astar_points) that is close to a given point.
        Args:
            point: A [x,y,z] array with the position of the point
        Returns:
            Index of the closest point in self.astar_points or False if no point found nearby.
        '''
        distance_to_existing_nodes = np.linalg.norm(self.points - point, axis=1)
        closest_node = np.argmin(distance_to_existing_nodes)
        distance = distance_to_existing_nodes[closest_node]
        if distance < self.step_size * 0.5:
            return closest_node

        return False
        

    def print_results(self):
        for idx, point in enumerate(self.points):
            self.coverable_pcd.visit_position(point, apply_unique=False)
        self.coverable_pcd.covered_points_idx = np.unique(self.coverable_pcd.covered_points_idx )
        self.print("coverage: " + str(self.coverable_pcd.get_coverage_efficiency()))
        self.coverable_pcd.covered_points_idx = np.array([])
        self.print("time: " + str(timeit.default_timer() - self.start_time))
