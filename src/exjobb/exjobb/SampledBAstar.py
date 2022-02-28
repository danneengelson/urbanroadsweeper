
import timeit
import numpy as np
import operator
import pickle

from exjobb.CPPSolver import CPPSolver, Segment
from exjobb.SampledBAStarSegment import BAStarSegment
from exjobb.SampledSpiralSegment import SpiralSegment
from exjobb.Parameters import BASTAR_STEP_SIZE, BASTAR_VISITED_TRESHOLD, RANDOM_BASTAR_VISITED_TRESHOLD, COVEREAGE_EFFICIENCY_GOAL, RANDOM_BASTAR_MAX_ITERATIONS, RANDOM_BASTAR_NUMBER_OF_ANGLES, RANDOM_BASTAR_PART_I_COVERAGE, RANDOM_BASTAR_MIN_COVERAGE, RANDOM_BASTAR_MIN_SPIRAL_LENGTH, RANDOM_BASTAR_VARIANT_DISTANCE, RANDOM_BASTAR_VARIANT_DISTANCE_PART_II, ROBOT_RADIUS, ROBOT_SIZE
from exjobb.PointCloud import PointCloud
from exjobb.Tree import Tree

DO_BASTAR_PLANNING = True
ONLY_PART_I = False

class SampledBAstar(CPPSolver):
    ''' Solving the Coverage Path Planning Problem with Random Sample BAstar with Inward Spiral
    '''
    def __init__(self, print, motion_planner, coverable_pcd, cost_function, time_limit=None, parameters=None):
        '''
        Args:
            print: function for printing messages
            motion_planner: Motion Planner of the robot wihch also has the Point Cloud
        '''
        self.print = print
        super().__init__(print, motion_planner, coverable_pcd, time_limit)
        self.name = "Random BAstar"
        self.cost_function = cost_function
        

        if parameters is None:
            self.max_distance = RANDOM_BASTAR_VARIANT_DISTANCE
            self.max_distance_part_II = RANDOM_BASTAR_VARIANT_DISTANCE_PART_II
            self.nbr_of_angles = RANDOM_BASTAR_NUMBER_OF_ANGLES
            self.ba_exploration = RANDOM_BASTAR_PART_I_COVERAGE
            self.coverage_2 = COVEREAGE_EFFICIENCY_GOAL
            self.min_spiral_coverage = RANDOM_BASTAR_MIN_SPIRAL_LENGTH
            self.min_bastar_coverage = RANDOM_BASTAR_MIN_COVERAGE
            self.max_iterations = RANDOM_BASTAR_MAX_ITERATIONS
            self.step_size = BASTAR_STEP_SIZE
            self.visited_threshold = BASTAR_VISITED_TRESHOLD
        else:
            self.max_distance = parameters["max_distance"]
            self.max_distance_part_II = parameters["max_distance_part_II"]
            self.ba_exploration = parameters["ba_exploration"]
            self.min_spiral_cost_per_coverage = parameters["min_spiral_cost_per_coverage"]
            self.min_bastar_cost_per_coverage = parameters["min_bastar_cost_per_coverage"]
            self.step_size = parameters["step_size"]
            self.visited_threshold = parameters["visited_threshold"]

        
        self.sampledbastar_stats = {}
        self.sampledbastar_stats_over_time = []
        self.all_movements = []


    def get_cpp_path(self, start_point, goal_coverage = None):
        """Generates a path that covers the area using BAstar Algorithm.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot

        Returns:
            Nx3 array with waypoints
        """
            
        self.start_tracking()
        self.move_to(start_point)
        self.all_segments = []
        self.visited_waypoints = np.empty((0,3))
        
        self.tmp_coverable_pcd = PointCloud(self.print, points=self.coverable_pcd.points)
        self.explored_pcd = PointCloud(self.print, points=self.coverable_pcd.points)
        self.uncovered_coverable_points_idx = np.arange(len(self.tmp_coverable_pcd.points))
        iter = 1

        if False:
            with open('cached_sampled_show.dictionary', 'rb') as cached_pcd_file:
                cache_data = pickle.load(cached_pcd_file)
                self.all_segments = cache_data["all_segments"]
        else:
            #### PART 1 - BA* ####    
            self.print("PART 1 - Covering with BA*")
            coverage = self.tmp_coverable_pcd.get_coverage_efficiency()        
            exploration = self.explored_pcd.get_coverage_efficiency()        

            while exploration < self.ba_exploration and coverage < goal_coverage and not self.time_limit_reached():
                iter += 1            
                random_point = self.get_random_uncovered_point(ignore_list = self.explored_pcd.covered_points_idx, iter=iter)
                #self.points_to_mark.append({
                #    "point": random_point,
                #    "color": [1.0,0.0,1.0]
                #})

                if random_point is False:
                    break
                
                closest_border_point, _ = self.find_closest_border(random_point, self.step_size, self.visited_threshold, self.visited_waypoints)
                self.points_to_mark.append({
                    "point": closest_border_point,
                    "color": [0.0,1.0,0.0]
                })
                
                
                BA_segments_from_point = []

                for angle_idx in range(4):

                    angle_offset = angle_idx * 2*np.pi/4
                    coverable_pcd = PointCloud(self.print, points=self.coverable_pcd.points)
                    new_BAstar_path = BAStarSegment(self.print, self.motion_planner, closest_border_point, angle_offset, self.visited_waypoints, coverable_pcd, self.max_distance, self.step_size, self.visited_threshold, self.time_left())
                    
                    BA_segments_from_point.append(new_BAstar_path)
                    
                
                

                accepted_segments = list(filter(lambda x: self.get_cost_per_coverage(x) < self.min_bastar_cost_per_coverage, BA_segments_from_point))

                #self.print([self.get_cost_per_coverage(a) for a in BA_segments_from_point])
                if not accepted_segments:
                    #coverable_pcd = PointCloud(self.print, points=self.coverable_pcd.points)
                    #spiral_segment = SampledSpiralSegment(self.print, self.motion_planner, closest_border_point, self.visited_waypoints, coverable_pcd, self.max_distance_part_II, self.step_size, self.visited_threshold, self.time_left())
                    #cost_per_coverage = self.get_cost_per_coverage(spiral_segment)
                    #self.print("cost_per_coverage spiral: " + str(cost_per_coverage))
                    #if cost_per_coverage < 15000:
                    #    self.add_segment(spiral_segment)
                    #    best_BA_segment = spiral_segment
                    #    self.print("COVERING WITH SPIRAL INSTEAD")
                    #else:
                    best_BA_segment = max(BA_segments_from_point, key=operator.attrgetter("coverage"))           
                    #return best_BA_segment.path
                else:
                    #best_BA_segment = max(BA_segments_from_point, key=operator.attrgetter("coverage"))
                    costs = [self.get_cost_per_coverage(segment) for segment in accepted_segments]
                    #self.print(costs)

                    best_BA_segment_idx = np.argmin(costs)  
                    best_BA_segment =  accepted_segments[best_BA_segment_idx]
                    self.add_segment(best_BA_segment)
                    coverage = self.tmp_coverable_pcd.get_coverage_efficiency()      
                    self.print_update(coverage)
                    self.sampledbastar_stats_over_time.append({
                        "time": timeit.default_timer() - self.start_time,
                        "coverage": coverage,
                        "iteration": iter,
                        "path": best_BA_segment.path, 
                        "segment": best_BA_segment
                    })
                    #return best_BA_segment.path
                    #return best_BA_segment.path
                self.print_segment_stats(best_BA_segment, "BA*", iter)
                self.explored_pcd.covered_points_idx = np.unique(np.append(self.explored_pcd.covered_points_idx, best_BA_segment.covered_points_idx))
                exploration = self.explored_pcd.get_coverage_efficiency()
                self.print("exploration: " + str(exploration))

                
                

            self.print("Number of found paths: " + str(len(self.all_segments)))


            self.sampledbastar_stats["Part1_segments"] = len(self.all_segments)
            self.sampledbastar_stats["Part1_coverage"] = coverage
            self.sampledbastar_stats["Part1_iterations"] = iter
            self.sampledbastar_stats["Part1_time"] = timeit.default_timer() - self.start_time

            

            #### PART 2 - Inward Spiral ####    
            self.print("PART 2 - Covering with Inward spiral")
            self.explored_pcd.covered_points_idx = self.tmp_coverable_pcd.covered_points_idx
            
            iter = 0
            while coverage < goal_coverage and not self.time_limit_reached(): 
                iter += 1

                random_point = self.get_random_uncovered_point()
                if random_point is False:
                    break

                closest_border_point, _ = self.find_closest_border(random_point, self.step_size, self.visited_threshold, self.visited_waypoints)
                coverable_pcd = PointCloud(self.print, points=self.coverable_pcd.points)
                spiral_segment = SpiralSegment(self.print, self.motion_planner, closest_border_point, self.visited_waypoints, coverable_pcd, self.max_distance_part_II, self.step_size, self.visited_threshold, self.time_left())
                
                self.print_segment_stats(spiral_segment, "Spiral", iter)
                #self.print(self.get_cost_per_coverage(spiral_segment))
                if self.get_cost_per_coverage(spiral_segment) > self.min_spiral_cost_per_coverage:

                    close_coverable_points_idx = spiral_segment.covered_points_idx
                    if not len(close_coverable_points_idx):
                        close_coverable_points_idx =  self.tmp_coverable_pcd.points_idx_in_radius(closest_border_point, self.visited_threshold)
                    self.uncovered_coverable_points_idx = self.delete_values(self.uncovered_coverable_points_idx, close_coverable_points_idx)
                    #self.print("Too short spiral")
                    continue


                
                self.add_segment(spiral_segment)
                coverage = self.tmp_coverable_pcd.get_coverage_efficiency() 
                self.sampledbastar_stats_over_time.append({
                        "time": timeit.default_timer() - self.start_time,
                        "coverage": coverage,
                        "iteration": iter,
                        "path": spiral_segment.path,
                        "segment": spiral_segment
                    })

                #self.print("length: " + str(len(spiral_segment.path)))
                self.print_update(coverage)
                #self.print("Uncovered points: " + str(len(self.uncovered_coverable_points_idx)))

            self.sampledbastar_stats["Part2_segments"] = len(self.all_segments) - self.sampledbastar_stats["Part1_segments"]
            self.sampledbastar_stats["Part2_coverage"] = coverage
            self.sampledbastar_stats["Part2_iterations"] = iter
            self.sampledbastar_stats["Part2_time"] = timeit.default_timer() - self.start_time
            

        # if False:
            with open('cached_sampled_show.dictionary', 'wb') as cached_pcd_file:
                cache_data = {  "all_segments": self.all_segments }
                pickle.dump(cache_data, cached_pcd_file)
            
        
        total = np.empty((0,3))
        for segment in self.all_segments:
            total = np.append(total, segment.path, axis=0)
        #return total
        paths_to_visit_in_order  = self.traveling_salesman(self.all_segments)

        self.follow_paths(start_point, paths_to_visit_in_order)

        #self.print_stats(self.path)
        self.print(self.sampledbastar_stats)

        return self.path


    def print_segment_stats(self, segment, type, iter):
        cost = self.get_cost_per_coverage(segment)
        self.print(str(iter) + " - " + type + " segment. Coverage: " + str(round(segment.coverage*100,3)) + ". cost per coverage: " + str(cost))


    def get_cost_per_coverage(self, segment):
        if segment.coverage == 0:
                return np.Inf
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

        length_of_path = get_length_of_path(segment.path)
        rotation = get_total_rotation(segment.path[:,0:2])
        return self.cost_function(length_of_path, rotation)/segment.coverage

    def traveling_salesman(self, paths):
        """Using Traveling Salesman Algorithm to order the path in an order
        that would minimise the total length of the path.

        Args:
            paths: List of paths of types SampledSpiralSegment and BAStarSegment

        Returns:
            Ordered list of paths
        """
        tree = Tree("BAstar paths")      
        start_nodes_idx = []  
        end_nodes_idx = []  
        
        def get_weight(from_idx, to_idx):
            from_point = tree.nodes[from_idx]
            to_point = tree.nodes[to_idx]
            return 100 + np.linalg.norm( to_point[0:2] - from_point[0:2] ) + 10 * abs( to_point[2] - from_point[2] )

        for path in paths:
            start_point = path.start
            end_point = path.end
            start_point_node_idx = tree.add_node(start_point)
            start_nodes_idx.append(start_point_node_idx)

            for node_idx, point in enumerate(tree.nodes[:-1]):
                tree.add_edge(start_point_node_idx, node_idx, get_weight(start_point_node_idx, node_idx))
            
            end_point_node_idx = tree.add_node(end_point)
            end_nodes_idx.append(end_point_node_idx)
            for node_idx, point in enumerate(tree.nodes[:-2]):
                tree.add_edge(end_point_node_idx, node_idx, get_weight(end_point_node_idx, node_idx))

            tree.add_edge(start_point_node_idx, end_point_node_idx, 0)

        traveling_Salesman_path =  tree.get_traveling_salesman_path()
        self.print(traveling_Salesman_path)

        paths_in_order = []
        current_position = np.array([0,0,0])

        for node_idx in traveling_Salesman_path:

            if np.array_equal(tree.nodes[node_idx], current_position):
                continue

            if node_idx in start_nodes_idx:
                path_idx = start_nodes_idx.index(node_idx)
                current_path = paths[path_idx]
                paths_in_order.append(current_path)
            elif node_idx in end_nodes_idx:
                path_idx = end_nodes_idx.index(node_idx)
                current_path = paths[path_idx]
                current_path.path = np.flip(current_path.path, 0)
                current_path.end = current_path.start
                current_path.start = tree.nodes[node_idx]
                paths_in_order.append(current_path)
            else:
                self.print("Not start or end point..")

            current_position = current_path.end

        return paths_in_order

        
    def add_segment(self, segment):
        self.all_segments.append(segment)
        self.visited_waypoints = np.append(self.visited_waypoints, segment.path, axis=0)
        self.tmp_coverable_pcd.covered_points_idx = np.unique(np.append(self.tmp_coverable_pcd.covered_points_idx, segment.covered_points_idx))
        self.uncovered_coverable_points_idx = self.delete_values(self.uncovered_coverable_points_idx, segment.covered_points_idx)


    def get_covered_points_idx_from_paths(self, paths):
        """Finds indices of all covered points by the given paths

        Args:
            paths: Generated Paths of class SampledBAstarSegment

        Returns:
            List of points indices that has been covered
        """
        covered_points_idx = np.array([])

        for path in paths:
            covered_points_idx = np.unique(np.append(covered_points_idx, path.covered_points_idx, axis=0))

        return covered_points_idx

    def follow_paths(self, start_position, paths_to_visit_in_order):
        """Connects all paths with Astar and make the robot walk through the paths.

        Args:
            start_position: A [x,y,z] np.array of the start position of the robot
            paths_to_visit_in_order:    Ordered list of paths of types SampledSpiralSegment 
                                        and BAStarSegment
        """
        current_position = start_position
        

        for idx, path in enumerate(paths_to_visit_in_order):
            
            self.print("Moving to start of path " + str(idx+1) + " out of " + str(len(paths_to_visit_in_order)))
            path_to_next_starting_point = self.motion_planner.Astar(current_position, path.start)
            self.follow_path(path_to_next_starting_point)
            self.path = np.append(self.path, path.path, axis=0)
            self.coverable_pcd.covered_points_idx = np.unique(np.append(self.coverable_pcd.covered_points_idx, path.covered_points_idx, axis=0))
            current_position = self.path[-1]
            self.all_movements.append(Segment(path_to_next_starting_point))

    def get_random_uncovered_point(self, ignore_list = None, iter = False ):
        """Returns a random uncovered point

        Args:
            visited_waypoints: Nx3 array with waypoints that has been visited
            iter (bool, optional): Integer for random seed. Defaults to False.

        Returns:
            A [x,y,z] position of an unvisited point.
        """
        uncovered_coverable_points_idx = self.uncovered_coverable_points_idx

        if iter is not False:
            np.random.seed(20*iter)

        if ignore_list is not None:
            uncovered_coverable_points_idx = self.delete_values(uncovered_coverable_points_idx, ignore_list)
        while len(uncovered_coverable_points_idx) and not self.time_limit_reached():
            random_idx = np.random.choice(len(uncovered_coverable_points_idx), 1, replace=False)[0]
            random_uncovered_coverable_point_idx = uncovered_coverable_points_idx[random_idx]
            random_uncovered_coverable_point = self.coverable_pcd.points[random_uncovered_coverable_point_idx]

            closest_traversable_point = self.traversable_pcd.find_k_nearest(random_uncovered_coverable_point, 1)[0]
            if self.has_been_visited(closest_traversable_point, self.visited_threshold,  self.visited_waypoints):
                close_coverable_points_idx = self.tmp_coverable_pcd.points_idx_in_radius(random_uncovered_coverable_point, ROBOT_RADIUS)
                self.uncovered_coverable_points_idx = self.delete_values(self.uncovered_coverable_points_idx, close_coverable_points_idx)
                #self.print("Has been visited. Removing " + str(len(close_coverable_points_idx)))
                closest_not_visited = self.find_closest_traversable(closest_traversable_point, self.step_size, self.visited_threshold, self.visited_waypoints, self.step_size*10)
                if closest_not_visited is False:
                    #self.print("BFS could not find an unvisited close")
                    continue 
                return closest_not_visited
                
                continue
            
            if not self.accessible(random_uncovered_coverable_point, self.visited_waypoints):
                close_coverable_points_idx = self.tmp_coverable_pcd.points_idx_in_radius(random_uncovered_coverable_point, ROBOT_RADIUS)
                self.uncovered_coverable_points_idx = self.delete_values(self.uncovered_coverable_points_idx, close_coverable_points_idx)
                #self.print("Inaccessible. Removing " + str(len(close_coverable_points_idx)))
                continue
            
            break

        if len(uncovered_coverable_points_idx) and not self.time_limit_reached():
            return closest_traversable_point
        
        return False




    def delete_values(self, array, values):
        ''' Removes specific values from an array with unique values
        Args:
            array: NumPy array with unique values to remove values from
            values: NumPy array with values that should be removed.
        '''
        return array[ np.isin(array, values, assume_unique=True, invert=True) ]

    def delete_values_not_unique(self, array, values):
        ''' Removes specific values from an array
        Args:
            array: NumPy array to remove values from
            values: NumPy array with values that should be removed.
        '''
        return array[ np.isin(array, values, invert=True) ]