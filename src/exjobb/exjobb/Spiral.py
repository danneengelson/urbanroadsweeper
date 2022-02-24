import numpy as np

from exjobb.CPPSolver import CPPSolver, Segment
from exjobb.MotionPlanner import MotionPlanner
from exjobb.Parameters import ROBOT_SIZE, COVEREAGE_EFFICIENCY_GOAL, SPIRAL_STEP_SIZE, SPIRAL_VISITED_TRESHOLD

class Segment():
    
    def __init__(self, path) -> None:
        self.path = path

class Spiral(CPPSolver):
    """ Implementation of the Inward Spiral Coverage Path Planning Algorithm
    """

    def __init__(self, print, motion_planner, coverable_pcd, time_limit=None, parameters=None):
        """
        Args:
            print: function for printing messages
            motion_planner: Motion Planner of the robot wihch also has the Point Cloud
        """
        
        self.print = print
        super().__init__(print, motion_planner, coverable_pcd, time_limit)
        self.name = "Inward Spiral"

        if parameters is None:
            self.step_size = SPIRAL_STEP_SIZE
            self.visited_threshold = SPIRAL_VISITED_TRESHOLD
        else:
            self.step_size = parameters["step_size"]  
            self.visited_threshold =  parameters["visited_threshold"]


    def get_cpp_path(self, start_point, angle_offset = None, goal_coverage=None):
        """Generates a path that covers the area using Inward Spiral Algorithm.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot

        Returns:
            Nx3 array with waypoints
        """
        if goal_coverage is None:
            goal_coverage = COVEREAGE_EFFICIENCY_GOAL

        self.print("step_size: " + str(self.step_size))
        self.print("visited_threshold: " + str(self.visited_threshold))

        self.start_tracking()
        self.move_to(start_point)

        


        next_starting_point, current_angle = self.find_closest_wall(start_point, self.step_size)
        path_to_next_starting_point = self.motion_planner.Astar(start_point, next_starting_point)
        self.follow_path(path_to_next_starting_point)

        current_position = next_starting_point
        coverage = 0

        
        self.all_segments = []
        self.all_movements = []
        
        iter = 0

        self.points_to_mark.append({
                "point": start_point,
                "color": [0.0,1.0,0.0]
        })

        while coverage < goal_coverage and not self.time_limit_reached():
            iter += 1

            next_starting_point, evaluated_points =  self.get_next_starting_point(current_position)

            
            
            if next_starting_point is False:
                if len(evaluated_points) == 1:
                    next_starting_point = self.path[-2]
                else:
                    break
            

            path_until_dead_zone, new_current_position = self.get_path_until_dead_zone(next_starting_point, current_angle)

            


            path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)
            
            while path_to_next_starting_point is False:
                current_position = self.step_back()
                path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)

            self.follow_path(path_to_next_starting_point)

            self.points_to_mark.append({
                "point": next_starting_point,
                "color": [1.0,1.0,0.0]
            })

            self.points_to_mark.append({
                "point": new_current_position,
                "color": [1.0,0.0,0.0]
            })

            self.follow_path(path_until_dead_zone)

            current_position = new_current_position
            current_angle = self.get_angle(self.path[-2], current_position)
            coverage = self.coverable_pcd.get_coverage_efficiency()
            self.save_sample_for_results(coverage)
            self.print_update(coverage)

            

            self.all_segments.append(Segment(path_until_dead_zone))
            self.all_movements.append(Segment(path_to_next_starting_point))
            
  
        #self.print_stats(self.path)
        
        return self.path


    

 
    def get_next_starting_point(self, start_position, max_distance = False):
        """Using Wavefront algorithm to find the closest obstacle free uncovered position.

        Args:
            start_position: A [x,y,z] np.array of the start position of the search

        Returns:
            An obstacle free uncovered position.
        """
        if not self.has_been_visited(start_position, self.visited_threshold):
            return start_position, [start_position]
            
        last_layer = np.array([start_position])
        visited = np.array([start_position])
        while len(last_layer):
            new_layer = np.empty((0,3))
            for pos in last_layer:
                
                

                neighbours = self.get_neighbours(pos, self.step_size)
                #self.print("neighbours" + str(neighbours))
                for neighbour in neighbours:
                    
                    
                    if max_distance and np.linalg.norm(start_position - neighbour) > max_distance:
                        continue

                    if self.has_been_visited(neighbour, self.visited_threshold, visited):
                        #self.print("visited")
                        continue                    

                    if not self.motion_planner.is_valid_step(pos, neighbour):
                        #self.print("invalid steo")
                        continue

                    if not self.has_been_visited(neighbour, self.visited_threshold) and self.accessible(neighbour, self.path):
                        #self.print("OK!")
                        return neighbour, visited
                    #self.print("unvisited lets go")
                    visited = np.append(visited, [neighbour], axis=0)
                    new_layer = np.append(new_layer, [neighbour], axis=0)

            last_layer = new_layer

        self.print("FAIL. No new uncovered obstacle free positions could be found from " + str(start_position))
        self.points_to_mark = [start_position]
        return False, visited

    def get_path_until_dead_zone(self, current_position, current_angle):
        """Covers the area in an inward spiral motion until  a dead zone is reached.

        Args:
            current_position: A [x,y,z] np.array of the start position of the search
            current_angle: A float value representing the starting angle in radians 

        Returns:
            New part of the path with waypoints and the position of the robot at the 
            end of the path.
        """

        local_path = np.array([current_position])
        dead_zone_reached = False
        while not dead_zone_reached:
            dead_zone_reached =  True
            neighbours = self.get_neighbours_for_spiral(current_position, current_angle)
            current_path = np.append(self.path, local_path, axis=0)

            for neighbour in neighbours: 
                
                if self.is_blocked(current_position, neighbour, self.visited_threshold, current_path) :
                    continue

                local_path = np.append(local_path, [neighbour], axis=0)
                current_angle = self.get_angle(current_position, neighbour)
                current_position = neighbour
                
                dead_zone_reached = False
                break

        return local_path, current_position




    def get_neighbours_for_spiral(self, current_position, current_angle):
        """Finds neighbours of a given position. And return them in the order
        to create the inward spiral motion.

        Args:
            current_position: A [x,y,z] np.array of the start position 
            current_angle: A float value representing the starting angle in radians

        Returns:
            List of neighbours in specific order to get the inward spiral motion,
        """
        right, forwardright, forward, forwardleft, left, backleft, back, backright = self.get_neighbours(current_position, self.step_size, current_angle)
        return [backright, right, forwardright, forward, forwardleft, left, backleft]






