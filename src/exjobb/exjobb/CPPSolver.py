from exjobb.Graph import TraversableGraph
from exjobb.PointCloud import PointCloud
from exjobb.Tree import Tree
import os
import numpy as np
from collections import deque
import timeit
import networkx as nx

import timeit
from heapq import heappush, heappop
import tracemalloc 

from networkx.algorithms.shortest_paths.weighted import _weight_function
from exjobb.MotionPlanner import MotionPlanner
from exjobb.Parameters import ROBOT_SIZE, ROBOT_RADIUS

ROBOT_RADIUS = ROBOT_SIZE/2
STEP_SIZE = ROBOT_SIZE
UNTRAVERSABLE_THRESHHOLD = 1.5*STEP_SIZE

class Segment():
    
    def __init__(self, path) -> None:
        self.path = path


class CPPSolver:
    ''' Abstract class of a Coverage Path Problem Solver
    '''

    def __init__(self, print, motion_planner, coverable_pcd, time_limit = None):
        '''
        Args:
            print: function for printing messages
            motion_planner: Motion Planner of the robot wihch also has the Point Cloud with traversable points.
            coverable_pcd: Point Cloud with points taht should be covered
        '''
        self.name = "General CPP"
        self.print = print
        self.traversable_pcd = motion_planner.traversable_pcd
        self.coverable_pcd = coverable_pcd
        self.motion_planner = motion_planner
        self.current_position = None
        self.path = np.empty((0,3))    
        self.points_to_mark = []
        self.time_limit = time_limit
        self.data_over_time = [{
            "time": 0,
            "path_point": 0,
            "coverage": 0
        }]

        

    def start_tracking(self):
        ''' Start the tracking of computational time and memory consumption
        '''
        tracemalloc.start()
        self.start_time = timeit.default_timer()

    def time_limit_reached(self):
        ''' Start the tracking of computational time and memory consumption
        '''
        if self.time_limit is None:
            return False
        current_time = timeit.default_timer() - self.start_time
        if current_time > self.time_limit:
            self.print("TIME LIMIT REACHED")
            return True
        return False

    def time_left(self):
        ''' Start the tracking of computational time and memory consumption
        '''
        if self.time_limit is None:
            return np.Inf
        current_time = timeit.default_timer() - self.start_time
        return self.time_limit - current_time
    
    def print_update(self, coverage):
        curr_time = str(round(timeit.default_timer() - self.start_time))
        self.print(self.name + " " + curr_time + "s: " + str(coverage*100))

    def print_stats(self, path, coverage = None):
        ''' Prints stats about the generated path
        Args:
            path: A Nx3 array with waypoints
        '''

        print("Done with planning. Calculating stats.")

        if not len(path):
            self.print("ERROR: Length of path is 0.")

        end_time = timeit.default_timer()
        snapshot = tracemalloc.take_snapshot()
        nbr_of_points_in_path = len(path)

        def get_memory_consumption(snapshot, key_type='lineno'):
            ''' Calculates memory consumption of the algorithm in KiB
            '''
            snapshot = snapshot.filter_traces((
                tracemalloc.Filter(False, "<frozen importlib._bootstrap>"),
                tracemalloc.Filter(False, "<unknown>"),
            ))
            top_stats = snapshot.statistics(key_type)
            total = sum(stat.size for stat in top_stats)
            return total / 1024
        
        def get_length_of_path(path):
            ''' Calculates length of the path in meters
            '''
            length = 0
            for point_idx in range(len(path) - 1):
                length += np.linalg.norm( path[point_idx] - path[point_idx + 1] )
            return length

        def get_total_rotation(path):
            ''' Calculates the total rotation made by the robot while executing the path
            '''
            rotation = 0

            for point_idx in range(len(path) - 2):
                prev = (path[point_idx+1] - path[point_idx]) / np.linalg.norm( path[point_idx] - path[point_idx + 1])
                next = (path[point_idx+2] - path[point_idx+1]) / np.linalg.norm( path[point_idx+2] - path[point_idx + 1])
                dot_product = np.dot(prev, next)
                curr_rotation = np.arccos(dot_product)
                if not np.isnan(curr_rotation):
                    rotation += abs(curr_rotation)

            return rotation

        length_of_path = get_length_of_path(path)
        rotation = get_total_rotation(path[:,0:2])
        #unessecary_coverage_mean = self.coverable_pcd.get_coverage_count_per_point(path)
        computational_time = end_time - self.start_time
        if coverage is None:
            coverage = self.coverable_pcd.get_coverage_efficiency()
        
        
        memory_consumption = get_memory_consumption(snapshot)

        print_text = "\n" + "=" * 20
        print_text += "\nAlgorithm: " + self.name
        print_text += "\nCoverage efficiency: " + str(round(coverage*100, 2)) + "%"
        print_text += "\nNumber of waypoints: " + str(nbr_of_points_in_path)
        print_text += "\nLength of path: " + str(round(length_of_path)) + " meter"
        print_text += "\nTotal rotation: " + str(round(rotation)) + " rad"
        #print_text += "\nVisits per point: " + str(unessecary_coverage_mean)
        print_text += "\nComputational time: " + str(round(computational_time, 1)) + " sec" 
        print_text += "\nMemory consumption: " + str(round(memory_consumption, 1)) + " KiB"

        

        print_text += "\n" + "=" * 20
        self.print(print_text)

        return {
            "Algorithm": self.name,
            "Coverage efficiency": round(coverage*100, 2),
            "Number of waypoints": nbr_of_points_in_path,
            "Length of path": round(length_of_path),
            "Total rotation": round(rotation),
            #"Visits per point": unessecary_coverage_mean,
            "Computational time": round(computational_time, 1),
            "Memory consumption": round(memory_consumption)
        }


    def follow_path(self, path):
        ''' Makes the robot follow a path. Marks points along the way as visited.
        Args:
            path: A Nx3 array with waypoints
        '''
        if path is False:
            return

        if len(path) > 0 and len(self.path) > 0:
            if np.array_equal(self.path[-1], path[0]):
                path = path[1:]

            if len(path) == 1:
                self.path = np.append( self.path, path, axis=0 )
                self.coverable_pcd.visit_path_to_position(path[0], self.path[-1]) 

            if len(path) > 1:
                new_path = np.append( [self.path[-1]], path, axis=0 )
                self.path = np.append( self.path, path, axis=0 )
                self.coverable_pcd.visit_path(new_path)            

    def move_to(self, position):
        ''' Makes the robot go to a specific position. Marks points along the way as visited.
        Args:
            position: A [x,y,z] array with the position
        '''
        if len(self.path) > 0:
            curr_position = self.path[-1]
            self.coverable_pcd.visit_path_to_position(position, curr_position)
        else:
            self.coverable_pcd.visit_position(position)
            self.print(len(self.coverable_pcd.covered_points_idx))
            
            
        self.path = np.append( self.path, [position], axis=0 )


    def find_closest_wall(self, start_position, step_size):
        """Using Breadth First Search to find the closest wall or obstacle.

        Args:
            start_position: A [x,y,z] np.array of the start position of the search
            step_size: The approximate distance of every step towards wall

        Returns:
            The position right before wall and the angle of the robot, which will be 
            along the wall.
        """
        queue = np.array([start_position])
        visited = np.array([start_position])
        while len(queue):
            current_position, queue = queue[0], queue[1:]
            neighbours = self.get_neighbours(current_position, step_size)
            for neighbour in neighbours:
                if not self.motion_planner.is_valid_step(current_position, neighbour):
                    current_angle = self.get_angle(start_position, current_position) + np.pi/2
                    return current_position, current_angle

                if self.has_been_visited(neighbour, path = visited):
                    continue
                
                queue = np.append(queue, [neighbour], axis=0)
                visited = np.append(visited, [neighbour], axis=0)

        return False, False
    
    def find_closest_border(self, start_position, step_size, visited_thereshold, path):
        """Using Breadth First Search to find the closest wall or obstacle.

        Args:
            start_position: A [x,y,z] np.array of the start position of the search
            step_size: The approximate distance of every step towards wall

        Returns:
            The position right before wall and the angle of the robot, which will be 
            along the wall.
        """
        queue = np.array([start_position])
        visited = np.array([start_position])
        while len(queue):
            current_position, queue = queue[0], queue[1:]
            neighbours = self.get_neighbours(current_position, step_size)
            for neighbour in neighbours:
                if not self.motion_planner.is_valid_step(current_position, neighbour) or self.has_been_visited(neighbour, visited_thereshold, path):
                    current_angle = self.get_angle(start_position, current_position) + np.pi/2
                    new_point  = self.motion_planner.new_point_at_angle(current_position, current_angle, step_size)
                    safety_iter = 0
                    while self.motion_planner.is_valid_step(current_position, new_point) and self.has_been_visited(new_point, visited_thereshold, path) and safety_iter < 20:
                        current_position = new_point
                        new_point  = self.motion_planner.new_point_at_angle(current_position, current_angle, step_size)
                        safety_iter += 1

                    return current_position, current_angle

                if self.has_been_visited(neighbour, path = visited):
                    continue
                
                queue = np.append(queue, [neighbour], axis=0)
                visited = np.append(visited, [neighbour], axis=0)

        return False, False

    def find_closest_traversable(self, start_position, step_size, visited_thereshold, path, max_distance=False):
        """Using Breadth First Search to find the closest wall or obstacle.

        Args:
            start_position: A [x,y,z] np.array of the start position of the search
            step_size: The approximate distance of every step towards wall

        Returns:
            The position right before wall and the angle of the robot, which will be 
            along the wall.
        """
        queue = np.array([start_position])
        visited = np.array([start_position])
        while len(queue):
            current_position, queue = queue[0], queue[1:]
            if max_distance and max_distance > np.linalg.norm(current_position - start_position):
                return False

            neighbours = self.get_neighbours(current_position, step_size)
            for neighbour in neighbours:
                if self.motion_planner.is_valid_step(current_position, neighbour) and not self.has_been_visited(neighbour, visited_thereshold, path):
                    return current_position

                if self.has_been_visited(neighbour, path = visited):
                    continue

                queue = np.append(queue, [neighbour], axis=0)
                visited = np.append(visited, [neighbour], axis=0)

        return False

    def get_neighbours(self, current_position, step_size, angle_offset = 0):
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
            angle = direction_idx/8*np.pi*2 + angle_offset

            x = current_position[0] + np.cos(angle) * step_size
            y = current_position[1] + np.sin(angle) * step_size
            z = current_position[2]
            pos = np.array([x, y, z])

            directions.append(self.traversable_pcd.find_k_nearest(pos, 1)[0])

        return directions

    def has_been_visited(self, point, visited_threshhold = 0.05, path=None):
        """Checks if a point has been visited. Looks if the distance to a point in the
        path is smaller than visited_threshhold.

        Args:
            point: A [x,y,z] np.array of the point that should be checked.
            path (optional): Specific path. Defaults to None.

        Returns:
            True if the point has been classified as visited
        """
        if path is None:
            path = self.path

        distances = np.linalg.norm(path - point, axis=1)
        return np.any(distances <= visited_threshhold) 

    def is_blocked(self, from_point, to_point, visited_threshhold, path = None):
        """Checks if a step is valid by looking if the end point has been visited 
        or is an obstacle.

        Args:
            from_point: A [x,y,z] np.array of the start position
            to_point: A [x,y,z] np.array of the end position
            path (optional): Specific path. Defaults to None.

        Returns:
            True if the point has been classified as blocked
        """

        if path is None:
            path = self.path

        if self.has_been_visited(to_point, visited_threshhold, path):
            return True
        
        if not self.motion_planner.is_valid_step(from_point, to_point):
            return True
        
        return False

    def is_node_blocked(self, from_node, to_node, path = None):
        """Checks if a step is valid by looking if the end point has been visited 
        or is an obstacle.

        Args:
            from_point: A [x,y,z] np.array of the start position
            to_point: A [x,y,z] np.array of the end position
            path (optional): Specific path. Defaults to None.

        Returns:
            True if the point has been classified as blocked
        """

        if to_node is False:
            return True

        if path is None:
            path = self.node_path

        if to_node in path:
            return True

        if to_node not in self.graph.neighbours[from_node].values():
            return True
        
        return False

    def accessible(self, point, visited_waypoints):
        ''' Checks if a point is accessible by trying to make a path from the point
        to the closest visited point.
        Args:
            point:  A [x,y,z] np.array of the point position
            visited_waypoints: A Nx3 NumPy array with positions that has been visited.
        Returns:
            True if the point is accessible.
        '''
        if len(visited_waypoints) == 0:
            return True
        closest_point = visited_waypoints[np.argmin(np.linalg.norm(visited_waypoints - point, axis=1))]
        path_to_point = self.motion_planner.RRT(closest_point, point)
        return path_to_point is not False
        
    def get_angle(self, from_pos, to_pos):
        """Calculates the 2D yaw angle of the robot after making a step

        Args:
            from_pos: A [x,y,z] np.array of the start position 
            to_pos: A [x,y,z] np.array of the end position

        Returns:
            An angle in radians
        """
        vec = to_pos[0:2] - from_pos[0:2]
        return np.angle( vec[0] + vec[1]*1j)

    def step_back(self):
        self.path, deleted_position = self.path[:-1], self.path[-1]
        self.coverable_pcd.unvisit_position(deleted_position)
        return self.path[-1]

    def save_sample_for_results(self, coverage = None):
        current_time = timeit.default_timer() - self.start_time
        if coverage is None:
            coverage = self.coverable_pcd.get_coverage_efficiency()
        current_path_length = len(self.path)
        self.data_over_time.append({
            "time": round(current_time, 1),
            "path_point": current_path_length,
            "coverage": round(coverage*100, 2)
        })