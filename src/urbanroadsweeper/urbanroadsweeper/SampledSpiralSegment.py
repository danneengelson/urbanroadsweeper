import numpy as np
from urbanroadsweeper.Parameters import ROBOT_SIZE
from urbanroadsweeper.Spiral import Spiral

class SpiralSegment(Spiral):
    """A class to generate a segment of BAstar path. Used in Sample-Based BAstar CPP Algorithm.
    """
    def __init__(self, print, motion_planner, starting_point, visited_waypoints, coverable_pcd, max_distance, step_size, visited_threshold, time_left):
        """
        Args:
            print: function for printing messages
            motion_planner:  Motion Planner of the robot wihch also has the Point Cloud
            starting_point: A [x,y,z] np.array of the start position of the robot
            angle_offset: An angle in radians, representing the primary direction of the paths
            visited_waypoints: A Nx3 array with points that has been visited and should be avoided
        """
        parameters = {
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        super().__init__(print, motion_planner, coverable_pcd, parameters)


        self.start = starting_point
        self.path = visited_waypoints
        self.path = np.append(self.path, [starting_point], axis=0)    
        self.start_tracking()
        self.time_limit = time_left

        self.new_path = np.empty((0,3))
        next_starting_point = starting_point
        
        current_angle = 0
        current_position = starting_point

        
        while not self.time_limit_reached():

            path_until_dead_zone, current_position = self.get_path_until_dead_zone(current_position, current_angle)
            self.follow_path(path_until_dead_zone)
            self.new_path = np.append(self.new_path, path_until_dead_zone, axis=0)     

            #self.print("path_until_dead_zone: " + str(path_until_dead_zone))
            #self.print("self.path: " + str(self.path))
            #self.print("current_position: " + str(current_position))
            

            next_starting_point, _ =  self.get_next_starting_point(current_position, max_distance)
            #self.print("next_starting_point: " + str(next_starting_point))
        
            if next_starting_point is False:
                break
            
            distance_to_point = np.linalg.norm(next_starting_point - current_position)

            if distance_to_point > max_distance:
                self.print("big dist")
                break

            path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)
             

            while path_to_next_starting_point is False:
                current_position = self.step_back()
                path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)

            

            self.follow_path(path_to_next_starting_point)
            self.new_path = np.append(self.new_path, path_to_next_starting_point, axis=0)     
            #self.print("path_to_next_starting_point: " + str(path_to_next_starting_point))
            
            current_position = self.path[-1]
            current_angle = self.get_angle(self.path[-2], current_position)


        self.end = current_position
        self.covered_points_idx = self.coverable_pcd.covered_points_idx
        self.coverage = self.coverable_pcd.get_coverage_efficiency()
        self.path = self.new_path

        self.coverable_pcd = None
        self.traversable_pcd = None
        self.motion_planner = None
        self.print = None
