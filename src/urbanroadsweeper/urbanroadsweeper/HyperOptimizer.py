import pickle
from hyperopt import fmin, tpe, hp, STATUS_OK, STATUS_FAIL, Trials
import numpy as np
from copy import deepcopy
import pprint
import timeit


from urbanroadsweeper.Parameters import ROBOT_SIZE

class HyptoOptimizer():


    def __init__(self, save, algorithm, print, hyper_start_pos, motion_planner, coverable_points, cost_function):
        self.current_algorithm = algorithm
        self.hyper_start_pos = hyper_start_pos
        self.motion_planner = motion_planner
        self.coverable_points = coverable_points
        self.print = print
        self.save = save
        self.best = None
        self.best_path = []
        self.cost_function = cost_function
        self.start_time = timeit.default_timer()
        self.eval_nr = 1

    def my_pprint(self, label, data):
        if type(data) == dict:
            print(label + ": ")
            for key, value in data.items():
                print(" "*4 + key + ": " + str(value))
        else:
            print(label + ": " + str(data))

    def get_random_angle(self):
        return np.pi*2 * np.random.randint(8) / 8

    def hyper_test(self, parameters):
        start_time = timeit.default_timer()
        cpp = self.current_algorithm["cpp"](self.print, self.motion_planner, self.coverable_points, self.current_algorithm["hyper_time_limit"], parameters)
        path = cpp.get_cpp_path(self.hyper_start_pos, goal_coverage=self.current_algorithm["hyper_min_coverage"]/100)
        stats = cpp.print_stats(cpp.path)
        loss = self.cost_function(stats["Length of path"], stats["Total rotation"])
        
        
        if stats["Coverage efficiency"] > self.current_algorithm["hyper_min_coverage"]:
            status = STATUS_OK
        else:
            status = STATUS_FAIL


        info = {
            "parameters": parameters,
            "stats": stats,
            "status": status,
            "cost": loss,
        }
        
        #self.current_algorithm["formatted_hyper_data"].append(info)

        

        if self.best is None or self.best["cost"] > loss:
            if status == STATUS_OK:
                self.best = info
                self.best_path = cpp.path

        print("-"*20)                   
        print("Current evaluation: ")
        print("-"*20)
        self.my_pprint("Parameters", info["parameters"])
        self.my_pprint("Stats from CPP Planning", info["stats"])
        self.my_pprint("Cost", info["cost"])
        if self.best is not None and self.best.get("cost"):
            self.my_pprint("Best accepted cost so far", self.best["cost"])
        else:
            print("No accepted cost so far")
        print("-"*20)
        self.my_pprint("Evaluation time consumption", str(round(timeit.default_timer()-start_time, 1)) + " sec")
        self.my_pprint("Total time consumption", str(round(timeit.default_timer()-self.start_time, 1)) + " sec")
        self.my_pprint("Average evaluation time consumption", str(round((timeit.default_timer()-self.start_time)/self.eval_nr, 1)) + " sec")
        
        print("="*40)
        self.save(info)
        self.eval_nr += 1

        return {
            'loss': loss,
            'status': status,
            'stats': stats,
        }    
    
    def hyper_opt_test(self, args, param_names):
        print("args", args)
        parameters = {}
        for idx, name in enumerate(param_names):
            if type(args) == tuple:
                parameters[name] = args[idx]
            else:
                parameters[name] = args
        return self.hyper_test(parameters)

    def hyper_test_inward_spiral(self, args): 
        step_size, visited_threshold = args
        parameters = {
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        return self.hyper_test(parameters)

    def hyper_test_bastar(self, args):
        angle_offset, step_size, visited_threshold = args
        parameters = {
            "angle_offset": angle_offset,
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        
        return self.hyper_test(parameters)
    
    def hyper_test_new(self, args): 
        param_1 = args
        parameters = {
            "param_1":  param_1,
        }
        return self.hyper_test(parameters)
        

    def hyper_test_sampled_bastar(self, args):
        ba_exploration, max_distance, max_distance_part_II, min_bastar_cost_per_coverage , min_spiral_cost_per_coverage, step_size, visited_threshold = args
        
        parameters = {
            "ba_exploration": ba_exploration,
            "max_distance": max_distance,
            "max_distance_part_II": max_distance_part_II,
            "min_spiral_cost_per_coverage": min_spiral_cost_per_coverage,
            "min_bastar_cost_per_coverage": min_bastar_cost_per_coverage,
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }        
        return self.hyper_test(parameters)


    def hyper_test_bfs(self, args):
        step_size, visited_threshold = args
        parameters = {
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        cpp = self.current_algorithm["cpp"](self.print, self.motion_planner, self.coverable_points, self.current_algorithm["hyper_time_limit"], parameters)
        path = cpp.breadth_first_search(self.hyper_start_pos, goal_coverage=self.current_algorithm["hyper_min_coverage"]/100)
        stats = cpp.print_results()
        loss = stats["no_of_nodes"]
        
        
        
        if stats["coverage"] > self.current_algorithm["hyper_min_coverage"]:
            status = STATUS_OK
        else:
            status = STATUS_FAIL
        
        info = {
            "parameters": parameters,
            "stats": stats,
            "status": status,
            "cost": loss
        }

        print({
            "parameters": parameters,
            "stats": stats,
            "status": status,
            "cost": loss
        })

        self.save(info)

        return {
            'loss': loss,
            'status': status,
            'stats': stats,
        }    
    def hyper_test_max_coverage(self, args):
        step_size, visited_threshold = args
        parameters = {
            "step_size":  step_size,
            "visited_threshold": visited_threshold
        }
        cpp = self.current_algorithm["cpp"](self.print, self.motion_planner, self.coverable_points, self.current_algorithm["hyper_time_limit"], parameters)
        path = cpp.breadth_first_search(self.hyper_start_pos, goal_coverage=1)
        stats = cpp.print_results()
        loss = 100 - stats["coverage"]
        
        
        
        #if stats["coverage"] > self.current_algorithm["hyper_min_coverage"]:
        #    status = STATUS_OK
        #else:
        #    status = STATUS_FAIL
        
        info = {
            "parameters": parameters,
            "stats": stats,
            "status": STATUS_OK,
            "cost": loss
        }

        print({
            "parameters": parameters,
            "stats": stats,
            "status": STATUS_OK,
            "cost": loss
        })

        self.save(info)

        return {
            'loss': loss,
            'status': STATUS_OK,
            'stats': stats,
        }   