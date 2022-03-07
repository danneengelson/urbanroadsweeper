import numpy as np
from urbanroadsweeper.CPPSolver import CPPSolver 

class NewCPP(CPPSolver):
    ''' Template to try yout own CPP
    '''
    def __init__(self, print, motion_planner, coverable_pcd, time_limit=None, parameters=None):
        '''
        Args:
            print: function for printing messages
            motion_planner: Motion Planner of the robot wihch also has the Point Cloud
        '''
        super().__init__(print, motion_planner, coverable_pcd, time_limit)
        self.name = "NewCPP"
        if parameters is None:
            #Set default parameters
            self.param_1 = 1
        else:
            self.param_1 = parameters["param_1"]     

        self.print("parameter values: " + str(parameters))   

    def get_cpp_path(self, start_point, goal_coverage=1):
        """Generates a path that covers the area using your algorithm.

        Args:
            start_point: A [x,y,z] np.array of the start position of the robot
            goal_coverage: A float nr between 0 and 1, deciding when to stop covering.

        Returns:
            Nx3 np.array with waypoints
        """

        self.start_tracking()
        coverage = 0
        self.move_to(start_point)
        current_position = start_point
        

        while coverage < goal_coverage and not self.time_limit_reached():
            next_starting_point = self.get_random_point()     
            path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)

            while path_to_next_starting_point is False:
                current_position = self.step_back()
                path_to_next_starting_point = self.motion_planner.Astar(current_position, next_starting_point)                

            self.follow_path(path_to_next_starting_point)
            current_position = next_starting_point
            coverage = self.coverable_pcd.get_coverage_efficiency()
            self.save_sample_for_results(coverage)
            self.print_update(coverage)
        
        return self.path

    def get_random_point(self):
        nbr_of_points_in_pcd = len(self.traversable_pcd.points) 
        return self.traversable_pcd.points[np.random.randint(nbr_of_points_in_pcd)]