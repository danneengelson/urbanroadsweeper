import open3d as o3d
import numpy as np
import pickle
from results_config import PCD_DATA, ALGORITHM_DATA, RESULTS
from itertools import groupby
from pprint import pprint
import sys
import matplotlib.pyplot as plt


import csv

def get_results(environment_name, algorithm_name):
    results_file = PCD_DATA[environment_name].get(algorithm_name)
    if not results_file:
        return None
    try: 
        with open(results_file, 'rb') as cached_pcd_file:
            return pickle.load(cached_pcd_file)            
    except IOError:
        print("Missing file: " + str(results_file))
        return None

def add_to_csv_file(file, dicts):
    with open(file, 'w', newline='') as csvfile:
        fieldnames = [i for i in dicts[0]]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for result in dicts:
            writer.writerow(result)

def main():

    # CPP parameters optimized using HyperOpt on start position.
    optimal_parameters = []
    algorithm_names = ALGORITHM_DATA.keys()
    for algorithm_name in algorithm_names:
        algorithm_label =  ALGORITHM_DATA[algorithm_name]["name"]
        # Find parameters
        for environment_name in PCD_DATA:
            results = get_results(environment_name, algorithm_name)
            if results is None:
                continue
            for parameter in results["opt_param"]:
                data = {
                        "Algorithm": algorithm_label,
                        "Parameter": parameter,
                }
                for environment_name in PCD_DATA:
                    environment_label = PCD_DATA[environment_name]["name"]
                    data[environment_label] = "N/A"

                optimal_parameters.append(data)
                        
            break
        
        # Find values of optimal parameters
        for environment_name in PCD_DATA:
            environment_label = PCD_DATA[environment_name]["name"]
            results = get_results(environment_name, algorithm_name)      
            if results is None:
                continue      
            for parameter in results["opt_param"]:
                value = results["opt_param"][parameter]
                for row in optimal_parameters:
                    if row["Algorithm"] == algorithm_label and row["Parameter"]== parameter:
                        row[environment_label] = value
                        break
    
    add_to_csv_file(RESULTS["opt_param_file"], optimal_parameters)

    # Result of meta-CPP domain adapted solution
    best_hyper_results = []
    for environment_name in PCD_DATA:
        environment_label = PCD_DATA[environment_name]["name"]
        for algorithm_name in algorithm_names: 
            algorithm_label =  ALGORITHM_DATA[algorithm_name]["name"]
            data = {
                "Environment": environment_label,
                "Algorithm": algorithm_label,
                "Cost": "N/A",
                "Length": "N/A",
                "Rotation": "N/A"
            }
            results = get_results(environment_name, algorithm_name)  
            if results is not None:
                if not results.get("best_hyper_stats"):
                    print("Missing best_hyper_stats in file: " + str((environment_name, algorithm_name)))
                else:
                    cost = results["best_hyper_stats"]["cost"]
                    length = results["best_hyper_stats"]["stats"]["Length of path"]
                    rotation = results["best_hyper_stats"]["stats"]["Total rotation"]
                    data["Cost"] = cost
                    data["Rotation"] = rotation
                    data["Length"] = length

            best_hyper_results.append(data)

    add_to_csv_file(RESULTS["opt_stats_file"], best_hyper_results)

    # Diagrams with Cost per Coverage
    
    for environment_name in PCD_DATA:
        environment_label = PCD_DATA[environment_name]["name"] 
        coverage_samples = np.arange(0, 101, 1)
        for algorithm_name in algorithm_names: 
            algorithm = ALGORITHM_DATA[algorithm_name]
            results = get_results(environment_name, algorithm_name)  
            if results is None:
                continue
            cost_values = []
            coverage_values = []
            for point_data in results["experiment_results"]:
                for sample_data in point_data:
                    cost_values.append(sample_data["length"] + sample_data["rotation"])
                    coverage_values.append(sample_data["coverage"])
            active_sample_values = coverage_samples[coverage_samples < np.max(coverage_values)]
            data = np.empty((0,len(active_sample_values)))
            for point_data in results["experiment_results"]:
                cost_values = np.array([data["length"] + data["rotation"] for data in point_data])
                coverage_values = np.array([data["coverage"] for data in point_data])
                interpolated_values = np.interp(active_sample_values, coverage_values, cost_values)
                data = np.append(data, [interpolated_values], axis=0)
            average, confidence_error = np.mean(data, axis=0), 2*np.std(data, axis=0)/np.sqrt(len(results["experiment_results"]))
            plt.plot(active_sample_values, average, algorithm["line"], label=algorithm["name"])
            plt.fill_between(active_sample_values, average-confidence_error, average+confidence_error, color=algorithm["confidence_color"])


        plt.ylabel('Cost')
        plt.xlabel('Coverage [%]')
        plt.legend()
        plt.title(environment_label + ': Cost (Length + Rotation) per Coverage')
        plt.xlim(0,100)
        plt.show()   

if __name__ == '__main__':
    main()