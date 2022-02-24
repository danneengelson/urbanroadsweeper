import numpy as np

from exjobb.CPPSolver import CPPSolver, Segment 
from exjobb.Parameters import BASTAR_STEP_SIZE, BASTAR_VISITED_TRESHOLD, COVEREAGE_EFFICIENCY_GOAL, ROBOT_SIZE
from exjobb.Graph import NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST
from exjobb.Graph import TraversableGraph


class BAstar(CPPSolver):
    ''' Solving the Coverage Path Planning Problem with BAstar
    '''
    def __init__(self, print, motion_planner, coverable_pcd, time_limit=None, parameters=None):
        '''
        Args:
            print: function for printing messages
            motion_planner: Motion Planner of the robot wihch also has the Point Cloud
        '''
        super().__init__(print, motion_planner, coverable_pcd, time_limit)
        self.name = "BAstar"
        if parameters is None:
            self.step_size = BASTAR_STEP_SIZE
            self.visited_threshold = BASTAR_VISITED_TRESHOLD
            self.angle_offset =  0
        else:
            self.step_size = parameters["step_size"]
            self.visited_threshold =  parameters["visited_threshold"]
            self.angle_offset =  parameters["angle_offset"]

        

    def get_cpp_path(self, start_point, angle_offset=None, goal_coverage=None):
        """Generates a path that covers the area using BAstar Algorithm.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            Nx3 array with waypoints
        """

        if angle_offset is None:
            angle_offset = self.angle_offset

        if goal_coverage is None:
            goal_coverage = COVEREAGE_EFFICIENCY_GOAL

        self.print("goal: " + str(goal_coverage))
        self.print("angle: " + str(angle_offset))
        
        self.start_tracking()
        coverage = 0
        self.move_to(start_point)

        
        

        current_position = start_point

        starting_point, _ = self.find_closest_wall(start_point, self.step_size)

        if starting_point is False:
            starting_point = current_position
        else:
            path_to_starting_point = self.motion_planner.Astar(current_position, starting_point)
            self.follow_path(path_to_starting_point)

        self.all_segments = []
        self.all_movements = []
        iter = 0

        while coverage < goal_coverage and not self.time_limit_reached():
            iter += 1
            self.points_to_mark.append({
                "point": starting_point,
                "color": [0.0,1.0,0.0]
            })

            
            path_to_cover_local_area, current_position = self.get_path_to_cover_local_area(starting_point, angle_offset)

            self.points_to_mark.append({
                "point": current_position,
                "color": [1.0,0.0,0.0]
            })

            if len(path_to_cover_local_area) == 0:
                self.print("No path found when covering local area!")  

            self.follow_path(path_to_cover_local_area)        
            next_starting_point = self.get_next_starting_point(self.path, angle_offset)     

            if next_starting_point is False:
                self.print("No next_starting_point found")
                break

            path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)

            while path_to_next_starting_point is False:
                current_position = self.step_back()
                path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)                

            self.follow_path(path_to_next_starting_point)
            starting_point = next_starting_point
            
            coverage = self.coverable_pcd.get_coverage_efficiency()
            self.save_sample_for_results(coverage)
            self.print_update(coverage)

            self.points_to_mark.append({
                "point": next_starting_point,
                "color": [1.0,1.0,0.0]
            })
            
            self.all_segments.append(Segment(path_to_cover_local_area))
            self.all_movements.append(Segment(path_to_next_starting_point))
        
        #self.print_stats(self.path)
        
        return self.path

    

    def get_cpp_node_path(self, start_point, angle_offset=0):
        """Generates a path that covers the area using BAstar Algorithm.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            Nx3 array with waypoints
        """
        self.graph = TraversableGraph(self.motion_planner, self.traversable_pcd, start_point)
        self.graph.points = self.graph.points
        self.graph.neighbours = self.graph.neighbours

        self.node_path = np.array([])

        self.start_tracking()
        self.move_to(start_point)
        coverage = 0
        start_node = self.graph.get_node(start_point)
        self.print("start_node" + str(start_node))
        self.print("start_node" + str(type(start_node)))
        self.node_path = np.array([start_node]).astype(int)

        current_node = start_node

        starting_point, _ = self.find_closest_wall(start_point, self.step_size)

        if starting_point is False:
            starting_node = start_node
        else:
            starting_node = self.graph.get_node(starting_point)
            path_to_starting_point = self.graph.astar(current_node, starting_node)
            self.node_path = np.append(self.node_path, path_to_starting_point)

        self.print("node_path: " + str(self.node_path))
        while coverage < self.goal_coverage and not self.time_limit_reached():
            
            path_to_cover_local_area, current_node = self.get_node_path_to_cover_local_area(starting_node, angle_offset)

            self.print("curr_node: " + str(current_node))

            if len(path_to_cover_local_area) == 0:
                self.print("No path found when covering local area!")  
                self.node_path = np.append(self.node_path, starting_node)

            #self.follow_path(path_to_cover_local_area)        

            self.node_path = np.append(self.node_path, path_to_cover_local_area)
            next_starting_node = self.get_next_starting_node(self.node_path, angle_offset)     
            self.print("next_starting_node: " + str(next_starting_node))
            if next_starting_node is False:
                self.print("No next_starting_point found")
                break
            
            path_to_next_starting_point = self.graph.astar(current_node, next_starting_node)

            if path_to_next_starting_point is False:
                self.print("AStar failed")
                break

            #while path_to_next_starting_point is False:
            #    current_position = self.step_back()
            #    path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)                

            self.node_path = np.append(self.node_path, path_to_next_starting_point)
            starting_node = next_starting_node
            
            #coverage = self.coverable_pcd.get_coverage_efficiency()
            #self.save_sample_for_results(coverage)
            coverage = len(np.unique(self.node_path)) / len(self.graph.points)
            self.print("coverage" + str(coverage))
        
        #self.print_stats(self.path)
        points_path = self.graph.points[self.node_path]
        #self.print(points_path)
        self.follow_path(points_path)
        #self.print(self.path)
        return points_path


    def get_path_to_cover_local_area(self, start_point, angle_offset = 0):
        """Generates BAstar paths to cover local area.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            Generated path with waypoints and the current position of the robot at the end
        """
       
        path_before = self.path

        current_position = start_point
        current_full_path = path_before
        critical_point_found = False
        local_path = np.empty((0,3))

        while not critical_point_found:
            critical_point_found = True
            neighbours = self.get_neighbours_for_bastar(current_position, angle_offset)
            for neighbour in neighbours:
                
                if self.is_blocked(current_position, neighbour, self.visited_threshold, current_full_path):
                    continue

                current_position = neighbour
                
                current_full_path = np.append( current_full_path, [neighbour], axis=0 )
                local_path = np.append( local_path, [neighbour], axis=0 )

                critical_point_found  = False

                break

        return local_path, current_position

    def get_node_path_to_cover_local_area(self, start_node, angle_offset = 0):
        """Generates BAstar paths to cover local area.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            Generated path with waypoints and the current position of the robot at the end
        """
       
        path_before = self.node_path

        current_node = start_node
        current_full_node_path = path_before
        critical_point_found = False
        local_node_path = np.array([]).astype(int)

        while not critical_point_found:
            critical_point_found = True
            neighbours = self.get_neighbours_nodes_for_bastar(current_node, angle_offset)
            #self.print("current_node" + str(current_node))
            #self.print("neighbours[current_node]" + str(self.graph.neighbours[current_node]))
            #self.print("neighbours" + str(neighbours))
            for neighbour in neighbours:

                if self.is_node_blocked(current_node, neighbour, current_full_node_path):
                    continue

                current_node = neighbour
                
                current_full_node_path = np.append( current_full_node_path, neighbour)
                local_node_path = np.append( local_node_path, neighbour)

                critical_point_found  = False

                break

        return local_node_path, current_node
        

    def get_next_starting_point(self, path, angle_offset = 0, lower_criteria = False):
        """Finds the next starting point by creating a backtrack list of possible points
        and choose the closest one.

        Args:
            path: Waypoint with the path that has been made so far in a Nx3 array
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            A position with an obstacle free uncovered point.
        """

        current_position = path[-1]
        distances = np.linalg.norm(path - current_position, axis=1)
        sorted_path_by_distance = path[np.argsort(distances)]
        
        for point in sorted_path_by_distance:
            def b(si, sj):
                                
                if not self.is_blocked(point, si, self.visited_threshold) and self.is_blocked(point, sj, self.visited_threshold):
                    return True

                if lower_criteria and not self.is_blocked(point, si, self.visited_threshold):
                    return True

                return False

            neighbours = self.get_neighbours(point, self.step_size, angle_offset)
            
            s1 = neighbours[0] #east
            s2 = neighbours[1] #northeast
            s3 = neighbours[2] #north
            s4 = neighbours[3] #northwest
            s5 = neighbours[4] #west
            s6 = neighbours[5] #southwest
            s7 = neighbours[6] #south
            s8 = neighbours[7] #southeast

            combinations =  [(s1, s8), (s1,s2), (s5,s6), (s5,s4), (s7,s6), (s7,s8)]
            for c in combinations:
                if b(c[0], c[1]):
                    return c[0]

        if not lower_criteria:
            self.print("WARNING: Lowered criteria")
            return self.get_next_starting_point(path, angle_offset, lower_criteria=True)
        return False

    def get_next_starting_node(self, node_path, angle_offset = 0, lower_criteria = False):
        """Finds the next starting point by creating a backtrack list of possible points
        and choose the closest one.

        Args:
            path: Waypoint with the path that has been made so far in a Nx3 array
            angle_offset (optional): Angle in radians of the main direction of the paths.

        Returns:
            A position with an obstacle free uncovered point.
        """

        current_node = int(node_path[-1])
        current_position = self.graph.points[current_node]
        distances = np.linalg.norm(self.graph.points[node_path] - current_position, axis=1)
        sorted_path_by_distance = node_path[np.argsort(distances)]
        
        for node in sorted_path_by_distance:

            def b(si, sj):
                                
                if not self.is_node_blocked(node, si) and self.is_node_blocked(node, sj):
                    return True

                if lower_criteria and not self.is_node_blocked(node, si):
                    return True

                return False

            neighbours = self.graph.neighbours[node]
            
            s1 = neighbours.get(0) #east
            s2 = neighbours.get(1) #northeast
            s3 = neighbours.get(2) #north
            s4 = neighbours.get(3) #northwest
            s5 = neighbours.get(4) #west
            s6 = neighbours.get(5) #southwest
            s7 = neighbours.get(6) #south
            s8 = neighbours.get(7) #southeast

            combinations =  [(s1, s8), (s1,s2), (s5,s6), (s5,s4), (s7,s6), (s7,s8)]
            for c in combinations:
                if b(c[0], c[1]):
                    return c[0]

        if not lower_criteria:
            self.print("WARNING: Lowered criteria")
            return self.get_next_starting_node(node_path, angle_offset, lower_criteria=True)

        return False


    def get_neighbours_for_bastar(self, current_position, angle_offset = 0):
        """Finds all neighbours of a given position and return them in the order
        to create the bastar zig-zag motion.

        Args:
            current_position: A [x,y,z] np.array of the start position 
            angle_offset: Angle offset in radians

        Returns:
            All 8 neighbours of the given position in order:
            north, south, northeast, northwest, southeast, southwest, east, west
        """

        east, northeast, north, northwest, west, southwest, south, southeast = self.get_neighbours(current_position, self.step_size, angle_offset)
        return [north, south, northeast, northwest, southeast, southwest, east, west]

    def get_neighbours_nodes_for_bastar(self, current_node, angle_offset = 0):
        """Finds all neighbours of a given position and return them in the order
        to create the bastar zig-zag motion.

        Args:
            current_position: A [x,y,z] np.array of the start position 
            angle_offset: Angle offset in radians

        Returns:
            All 8 neighbours of the given position in order:
            north, south, northeast, northwest, southeast, southwest, east, west
        """

        neighbours = self.graph.neighbours[current_node]
        priority = [NORTH, SOUTH, NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST, EAST, WEST]
        sorted_neighbours = []
        for dir in priority:
            if neighbours.get(dir):
                sorted_neighbours.append(neighbours[dir])
        return sorted_neighbours