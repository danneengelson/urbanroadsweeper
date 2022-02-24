import numpy as np
import timeit
from exjobb.PointCloud import PointCloud


class Experimenter():

    def __init__(self, algorithm, print):
        self.algorithm = algorithm
        self.results = []
        self.print = print
        pass

    def get_length_of_path(self, path):
        ''' Calculates length of the path in meters
        '''
        if len(path) < 2:
            return 0

        length = 0
        for point_idx in range(len(path) - 1):
            length += np.linalg.norm( path[point_idx] - path[point_idx + 1] )
        return length

    def get_total_rotation(self, path):
        ''' Calculates the total rotation made by the robot while executing the path
        '''
        if len(path) < 3:
            return 0
        
        rotation = 0
        for point_idx in range(len(path) - 2):
            prev = (path[point_idx+1] - path[point_idx]) / np.linalg.norm( path[point_idx] - path[point_idx + 1])
            next = (path[point_idx+2] - path[point_idx+1]) / np.linalg.norm( path[point_idx+2] - path[point_idx + 1])
            dot_product = np.dot(prev, next)
            curr_rotation = np.arccos(dot_product)
            if not np.isnan(curr_rotation):
                rotation += abs(curr_rotation)

        return rotation

    def get_coverage(self, cpp, path):
        tmp_pcd = PointCloud(self.print, points= cpp.coverable_pcd.points)
        tmp_pcd.visit_path(path)
        return tmp_pcd.get_coverage_efficiency()

    def perform_cpp(self, cpp, start_point, start_point_nr):

        path = cpp.get_cpp_path(start_point, goal_coverage=1)

        for sample in cpp.data_over_time:
            time = sample['time']
            coveage = sample["coverage"]
            length = self.get_length_of_path(path[0:sample["path_point"]])
            rotation = self.get_total_rotation(path[0:sample["path_point"]])
            stats = {
                "algorithm": self.algorithm,
                "point": str(start_point_nr) + " - " + str(start_point),
                "time": time,
                "coverage": coveage,
                "length": round(length),
                "rotation": round(rotation),
            }
            self.results.append(stats)
        print(stats)
        print(stats["algorithm"] + " done.")

    def perform_sample_cpp(self, cpp, start_point, start_point_nr):

        def follow_paths(cpp, start_position, paths_to_visit_in_order):
            current_position = start_position
            total_path = np.empty((0,3))
            for idx, path in enumerate(paths_to_visit_in_order):
                self.print("TSP - Moving to start of path " + str(idx+1) + " out of " + str(len(paths_to_visit_in_order)))
                path_to_next_starting_point = cpp.motion_planner.Astar(current_position, path.start)
                path_segment = path.path
                total_path = np.append(total_path, path_to_next_starting_point, axis=0)
                total_path = np.append(total_path, path_segment, axis=0)
                current_position = total_path[-1]
            return total_path

        path = cpp.get_cpp_path(start_point, goal_coverage=1)
        final_stats = cpp.print_stats(path)
        self.results.append({
                "algorithm": self.algorithm,
                "point": str(start_point_nr) + " - " + str(start_point),
                "time": 0,
                "coverage": 0,
                "length": 0,
                "rotation": 0,
            })

        for coverage_percent in np.arange(0.1, final_stats["Coverage efficiency"]/100, 0.1):
            paths_so_far = list(filter(lambda i: i['coverage'] <= coverage_percent, cpp.sampledbastar_stats_over_time))
            if paths_so_far:
                start_time = timeit.default_timer()
                Paths = [paths["segment"] for paths in paths_so_far]
                paths_to_visit_in_order  = cpp.traveling_salesman(Paths)
                total_path = follow_paths(cpp, start_point, paths_to_visit_in_order)
                
                time = max(paths_so_far, key=lambda path: path["time"])["time"] + timeit.default_timer() - start_time
                length = self.get_length_of_path(total_path)
                rotation = self.get_total_rotation(total_path)

                stats = {
                    "algorithm": self.algorithm,
                    "point": str(start_point_nr) + " - " + str(start_point),
                    "time": time,
                    "coverage": self.get_coverage(cpp, total_path)*100,
                    "length": round(length),
                    "rotation": round(rotation),
                }
                self.results.append(stats)
            
        #print(cpp.sampledbastar_stats_over_time)
        
        time = final_stats["Computational time"]
        coverage = final_stats["Coverage efficiency"]
        length = final_stats["Length of path"]
        rotation = final_stats["Total rotation"]
        self.results.append({
                "algorithm": self.algorithm,
                "point": str(start_point_nr) + " - " + str(start_point),
                "time": time,
                "coverage": coverage,
                "length": round(length),
                "rotation": round(rotation),
        })
        self.sample_specific_stats = cpp.sampledbastar_stats 
        print(final_stats)
        print(stats["algorithm"] + " done.")