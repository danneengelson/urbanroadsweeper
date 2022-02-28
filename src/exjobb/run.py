import sys
from exjobb.Environments import PointCloudEnvironment
from exjobb.MotionPlanner import MotionPlanner
from run_config import HYPER_MAX_EVAL, NUMBER_OF_START_POINTS, PRINT, ENVIRONMENT, ALGORITHMS, MANUAL_PARAMETERS, USE_MANUAL_PARAMETERS
from exjobb.Experimenter import Experimenter
from exjobb.HyperOptimizer import HyptoOptimizer
from hyperopt import fmin, tpe, hp, Trials
from copy import deepcopy
import pickle
from pprint import pprint
import numpy as np

def my_print(text):
        if PRINT:
            return print(text)
        else:
            return 

class RunScript():

    def save_data(self, formatted_hyper_data=None):
        with open(self.results_file, 'wb') as cached_pcd_file:
            if formatted_hyper_data:
                self.results["formatted_hyper_data"].append(formatted_hyper_data)
            pickle.dump(self.results, cached_pcd_file) 

    def main(self):
        pcd_name = sys.argv[1]
        pcd_file_name = ENVIRONMENT[pcd_name]["file"]
        algorithm_name = sys.argv[2]
        self.results_file = pcd_file_name + "_" + algorithm_name + ".dictionary"
                
        # Classify Point Cloud
        environment = PointCloudEnvironment(my_print, pcd_file_name + ".dictionary", pcd_file_name)
        coverable_points = environment.coverable_pcd.points
        traversable_points = environment.traversable_pcd.points 
         
        # Setup Motion Planner
        motion_planner = MotionPlanner(my_print, environment.traversable_pcd)

        # Experiment 1: Tune Parameters for best solution for one starting point
        if algorithm_name == "bastar":
            algorithm = ALGORITHMS["BA*"]
        if algorithm_name == "spiral":
            algorithm = ALGORITHMS["Inward Spiral"]
        if algorithm_name == "sampled":
            algorithm = ALGORITHMS["Sampled BA*"]

        self.results = {
            "name": algorithm["name"],
            "setup": {
                "hyper_time_limit": algorithm["hyper_time_limit"],
                "hyper_min_coverage": algorithm["hyper_min_coverage"],
                "experiment_time_limit": algorithm["experiment_time_limit"],
                "environment": ENVIRONMENT[pcd_name],
                "hyper_max_eval": HYPER_MAX_EVAL,
                "nbr_of_start_points": NUMBER_OF_START_POINTS,
            },
            "experiment_results": [],
            "hyper_data": [],
            "formatted_hyper_data": [],
            "sample_specific_stats": [],
            "best_hyper_path": [],
            "best_hyper_stats": []
        }

        if USE_MANUAL_PARAMETERS:
            opt_param = MANUAL_PARAMETERS[algorithm_name]
            if algorithm_name == "sampled":
                opt_param["min_bastar_cost_per_coverage"] = opt_param["min_bastar_cost_per_coverage"][pcd_name]
                opt_param["min_spiral_cost_per_coverage"] = opt_param["min_spiral_cost_per_coverage"][pcd_name]
            print("Using manually chosen parameters: ")
            pprint(opt_param)
            start_pos = ENVIRONMENT[pcd_name]["hyper_start_point"]
            cpp = algorithm["cpp"](print, motion_planner, coverable_points, algorithm["hyper_time_limit"], opt_param)
            path = cpp.get_cpp_path(start_pos, goal_coverage=algorithm["hyper_min_coverage"]/100)
            stats = cpp.print_stats(cpp.path)
            loss = stats["Total rotation"] + stats["Length of path"]

            info = {
                "parameters": opt_param,
                "stats": stats,
                "status": 'ok',
                "cost": loss
            }

            print("Results: ")
            pprint(info)

            self.results["opt_param"] = opt_param
            self.results["best_hyper_path"] = cpp.path
            self.results["best_hyper_stats"] = info
            self.save_data()
                    
        else: 
            trials = Trials()
            hyper_optimizer = HyptoOptimizer(self.save_data, algorithm, my_print, ENVIRONMENT[pcd_name]["hyper_start_point"], motion_planner, coverable_points)
            if algorithm_name == "bastar":
                opt_param = fmin(   hyper_optimizer.hyper_test_bastar,
                                    space=( hp.uniform('angle_offset', 0, np.pi*2),
                                            hp.uniform('step_size', 0.5, 1), 
                                            hp.uniform('visited_threshold', 0.25, 0.5)),
                                    algo=tpe.suggest,
                                    max_evals=HYPER_MAX_EVAL,
                                    trials=trials)
            elif algorithm_name == "spiral":
                opt_param = fmin(   hyper_optimizer.hyper_test_inward_spiral,
                                    space=( hp.uniform('step_size', 0.5, 1), 
                                            hp.uniform('visited_threshold', 0.25, 0.5)),
                                    algo=tpe.suggest,
                                    max_evals=HYPER_MAX_EVAL,
                                    trials=trials)
            elif algorithm_name == "sampled":
                coverage_2 = algorithm["hyper_min_coverage"]/100
                with open( ENVIRONMENT[pcd_name]["terrain_assessment_file"], 'rb') as cached_pcd_file:
                    terrain_assessment = pickle.load(cached_pcd_file)
                    cell_side = terrain_assessment["stats"]["Parameters"]["CELL_SIZE"]
                    coverable_cells = 0
                    for floor in terrain_assessment["stats"]["Floors"]:
                        coverable_cells += floor['Cell classification']["Number of COVERABLE cells"]
                    area = coverable_cells*cell_side*cell_side
                opt_param = fmin(   hyper_optimizer.hyper_test_newest_sampled_bastar_param,
                                    space=( hp.uniform('ba_exploration', 0.75, 0.95), 
                                        hp.uniform('max_distance', 1, 5),  
                                        hp.uniform('max_distance_part_II', 4, 10),
                                        hp.uniform('min_bastar_cost_per_coverage', 2.63*area, 5.26*area), 
                                        hp.uniform('min_spiral_cost_per_coverage', 5.26*area, 10.52*area), 
                                        hp.uniform('step_size', 0.5, 1.0), 
                                        hp.uniform('visited_threshold', 0.25, 0.5)
                                        ),
                                    algo=tpe.suggest,
                                    max_evals=HYPER_MAX_EVAL,
                                    trials=trials)
            print(trials.statuses())
            self.results["opt_param"] = opt_param
            self.results["hyper_data"] = trials.trials
            self.results["best_hyper_path"] = hyper_optimizer.best_path
            self.results["best_hyper_stats"] = hyper_optimizer.best
            self.save_data()

        print("results", self.results)
        print("opt_param", self.results["opt_param"])
        # Experiment 2: Robustness evaluation for different starting points
        for start_point_nr in range(NUMBER_OF_START_POINTS):
            start_point = ENVIRONMENT[pcd_name]["start_points"][start_point_nr]
            print("Start point " + str(start_point_nr) + ": " + str(start_point))

            if algorithm["do_experiment"]:                
                experimenter = Experimenter(algorithm["name"], print)
                parameters = None
                if "opt_param" in self.results:
                    parameters = self.results["opt_param"]

                print("parameters", parameters)
                cpp = algorithm["cpp"](my_print, motion_planner, coverable_points, algorithm["experiment_time_limit"], parameters)

                if "sample_specific_stats" in algorithm:
                    experimenter.perform_sample_cpp(cpp, start_point, start_point_nr)
                    self.results["sample_specific_stats"].append(experimenter.sample_specific_stats)
                else:
                    experimenter.perform_cpp(cpp, start_point, start_point_nr)

                self.results["experiment_results"].append(experimenter.results)
                self.save_data()


if __name__ == '__main__':
    script = RunScript()
    script.main()