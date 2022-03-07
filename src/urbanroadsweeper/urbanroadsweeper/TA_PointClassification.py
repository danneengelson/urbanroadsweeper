import numpy as np
import timeit
from urbanroadsweeper.PointCloud import PointCloud
from urbanroadsweeper.Parameters import ROBOT_SIZE, CELL_SIZE
from urbanroadsweeper.Tree import Tree
from urbanroadsweeper.MotionPlanner import MotionPlanner



class PointClassification():
    ''' Class for calculating points where a robot could go without collision.
    '''

    def __init__(self, print, pcd, param):
        ''' 
        Args:
            print: function for printing messages
            pcd: Point Cloud of the environment.
        '''
        self.pcd = pcd     
        self.print = print       
        self.param = param 

    def get_classified_points(self, ground_points_idx, uncoverable_border_points):
        ''' Given some points, find those which are traversable by the robot,
        given robot size specifics. go through every point in the point cloud 
        that is classified as obstacle and filter out ground points that are 
        close.
        Args:
            ground_points_idx:  indexes of points in the point cloud that are 
                                classified as obstacle free.
        '''
        start = timeit.default_timer()

        self.ground_point_cloud = PointCloud(self.print, points= self.pcd.points[ground_points_idx])
        traversable_points_idx = ground_points_idx
        false_uncoverable_idx = []
        self.print("Starting search...")
        for i, uncoverable_point in enumerate(uncoverable_border_points):
            #self.print("Working on " + str(i) + " out of " + str(len(uncoverable_border_points))) 
            distance_to_nearest_ground_point = self.ground_point_cloud.distance_to_nearest(uncoverable_point)
            if distance_to_nearest_ground_point < CELL_SIZE/2:
                false_uncoverable_idx.append(i)
        uncoverable_border_points = np.delete(uncoverable_border_points, false_uncoverable_idx, 0)



        for i, untraversable_point in enumerate(uncoverable_border_points):
            #if i % 100 == 0:
            #    self.print("Working on border pos " + str(i) + " out of " + str(len(uncoverable_border_points))) 
            collision_risk_points = self.pcd.points_idx_in_radius(untraversable_point, np.sqrt(1/2)*CELL_SIZE + 0.5*ROBOT_SIZE)
            traversable_points_idx = self.delete_values(traversable_points_idx, collision_risk_points)


        #Hindsight wrong classified points removal by hand:
        wrong_points = np.empty((0,3))
        
        #For pointcloud.pcd:
        #wrong_points = np.append(wrong_points, [[24.395610809326172, 12.705216407775879, -5.311060428619385]], axis=0)
        #wrong_points = np.append(wrong_points, [[-17.590679168701172, -3.7045161724090576, -6.118121147155762]], axis=0)
        #For bridge.pcd:
        #wrong_points = np.append(wrong_points, [[-98.5624, 182.8, -30.83]], axis=0)
        #wrong_points =  np.append(wrong_points, [[ 0.8125    ,  93.30000305, -32.33319855]], axis=0)
        #wrong_points = np.append(wrong_points, [[-17.590679168701172, -3.7045161724090576, -6.118121147155762]], axis=0)
        #for cross-pcd:
        wrong_points = np.append(wrong_points, [[-4.5,         1.5 ,       -0.18380356]], axis=0)
        
        wrong_points = self.param["FALSE_TRAVERSABLE_POINTS"]

        for wrong_point in wrong_points:
            points_nearby = self.pcd.points_idx_in_radius(wrong_point, 0.5*ROBOT_SIZE)
            traversable_points_idx = self.delete_values(traversable_points_idx, points_nearby)

        traversable_pcd = PointCloud(self.print, points= self.pcd.points[traversable_points_idx.astype(int)])

        


        coverable_points_idx_queue = ground_points_idx
        coverable_points_idx_queue = self.delete_values(coverable_points_idx_queue, traversable_points_idx)
        false_coverable_points_idx = np.array([])
        while len(coverable_points_idx_queue):
            #if len(coverable_points_idx_queue) % 1000 == 0:
            #    self.print("coverable_points_idx_queue: " + str(len(coverable_points_idx_queue)))
            point_idx, coverable_points_idx_queue = coverable_points_idx_queue[0], coverable_points_idx_queue[1:]
            point = self.pcd.points[point_idx]
            distance_to_nearest_traversable_point = traversable_pcd.distance_to_nearest(self.pcd.points[point_idx]) 
            if distance_to_nearest_traversable_point > ROBOT_SIZE/2:
                false_coverable_points_idx = np.append(false_coverable_points_idx, point_idx)
        
        real_coverable_points_idx = self.delete_values(ground_points_idx, false_coverable_points_idx)

        

        stats = self.print_result(start, len(real_coverable_points_idx), len(traversable_points_idx), len(false_coverable_points_idx), len(self.pcd.points))  

        return self.pcd.points[traversable_points_idx], self.pcd.points[real_coverable_points_idx], self.pcd.points[false_coverable_points_idx.astype(int)], stats


    def delete_values(self, array, values):
        ''' Removes specific values from an array
        Args:
            array: NumPy array to remove values from
            values: NumPy array with values that should be removed.
        '''
        return array[ np.isin(array, values, assume_unique=True, invert=True) ]

    def print_result(self, start, nbr_of_coverable, nbr_of_traversable, nbr_of_inaccessable, total_nbr_of_points):
        ''' Prints result data of the point classification.
        '''
        end = timeit.default_timer()
        self.print("="*20)
        self.print("POINT CLASSIFCATION")
        self.print("Computational time: " + str(round(end - start, 1)) + " sec")
        self.print("Number of COVERABLE points: " + str(nbr_of_coverable))   
        self.print("Number of TRAVERSABLE points: " + str(nbr_of_traversable))  
        self.print("Number of INACCESSABLE points: " + str(nbr_of_inaccessable))  
        self.print("Number of OBSTACLE points: " + str(total_nbr_of_points - nbr_of_inaccessable- nbr_of_coverable))          
        self.print("TOTAL Number of points: " + str(total_nbr_of_points))   

        return {
            "Computational time": round(end - start, 1),
            "Number of COVERABLE points":  nbr_of_coverable,   
            "Number of TRAVERSABLE points":  nbr_of_traversable,
            "Number of INACCESSABLE points":  nbr_of_inaccessable,  
            "Number of OBSTACLE points":  total_nbr_of_points - nbr_of_inaccessable- nbr_of_coverable,         
            "TOTAL Number of points": total_nbr_of_points 
        }