import numpy as np
from exjobb.BAstar import BAstar
from exjobb.Parameters import ROBOT_SIZE

class BAStarSegment(BAstar):
    """A class to generate a segment of BAstar path. Used in Sample-Based BAstar CPP Algorithm.
    """
    def __init__(self, print, motion_planner, starting_point, angle_offset, visited_waypoints, coverable_pcd, max_distance, step_size, visited_threshold, time_left):
        """
        Args:
            print: function for printing messages
            motion_planner:  Motion Planner of the robot wihch also has the Point Cloud
            starting_point: A [x,y,z] np.array of the start position of the robot
            angle_offset: An angle in radians, representing the primary direction of the paths
            visited_waypoints: A Nx3 array with points that has been visited and should be avoided
        """
        parameters = {
            "angle_offset": angle_offset,
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        super().__init__(print, motion_planner, coverable_pcd, parameters)

        self.start_tracking()
        self.time_limit = time_left
        self.start = starting_point
        self.path = visited_waypoints
        self.path = np.append(self.path, [starting_point], axis=0)    

        self.new_path = np.empty((0,3))
        next_starting_point = starting_point
        current_position = starting_point
        
        while not self.time_limit_reached():
           
            path_to_cover_local_area, current_position = self.get_path_to_cover_local_area(next_starting_point, angle_offset)
            
            #if len(path_to_cover_local_area) == 0:
            #    break


            self.follow_path(path_to_cover_local_area)   
            self.new_path = np.append(self.new_path, path_to_cover_local_area, axis=0)     


            next_starting_point = self.get_next_starting_point(self.path, angle_offset)   

            if next_starting_point is False:
                break

            distance_to_point = np.linalg.norm(next_starting_point - current_position)
            
            if distance_to_point > max_distance:
                break
            

            path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)
            
            while path_to_next_starting_point is False:
                current_position = self.step_back()
                path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)

            self.follow_path(path_to_next_starting_point)
            self.new_path = np.append(self.new_path, path_to_next_starting_point, axis=0)  

            
        
        self.end = current_position
        self.covered_points_idx = self.coverable_pcd.covered_points_idx
        self.coverage = self.coverable_pcd.get_coverage_efficiency()
        self.path = self.new_path

        self.coverable_pcd = None
        self.traversable_pcd = None
        self.motion_planner = None
        self.print = None
    
    def get_cost_per_coverage(self):
        if self.coverage == 0:
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

        length_of_path = get_length_of_path(self.path)
        rotation = get_total_rotation(self.path[:,0:2])
        return (length_of_path + rotation)/self.coverage