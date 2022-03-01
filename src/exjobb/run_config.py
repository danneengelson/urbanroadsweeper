
import numpy as np
from exjobb.BAstar import BAstar
from exjobb.Spiral import Spiral
from exjobb.SampledBAstar import SampledBAstar
from exjobb.PointCloud import PointCloud

HYPER_MAX_EVAL = 1 #100
NUMBER_OF_START_POINTS = 3 #10
HYPER_TIME_LIMIT = 20 #250
HYPER_MIN_COVERAGE = 20 #95
EXP_TIME_LIMIT = 10 #400
COST_FUNCTION = {
    "length": 2, #1
    "rotation": 1 #1
}

#By setting this to True no hyper evaluation will be made
USE_MANUAL_PARAMETERS = False 

PRINT = False

def cost(length, rotation):
    return COST_FUNCTION["length"]*length + COST_FUNCTION["rotation"]*rotation

MANUAL_PARAMETERS = {
    "spiral": {  
        'step_size': 0.75,
        'visited_threshold': 0.375
    },
    "bastar": {
        'angle_offset': 0,
        'step_size': 0.75,
        'visited_threshold': 0.375
    },
    "sampled": {
        'ba_exploration': 0.85,
        'max_distance': 3,
        'max_distance_part_II': 7,
        'min_bastar_cost_per_coverage': {
                "garage": 7504.37625*1.5, 
                "bridge": 10680.10125*1.5,
                "crossing": 5111.73375*1.5,
        },
        'min_spiral_cost_per_coverage': {
                "garage": 15008.7525*1.5,
                "bridge": 21360.2025*1.5,
                "crossing": 10223.4675*1.5, 
        },
        'step_size': 0.75,
        'visited_threshold': 0.375
    }

}

ENVIRONMENT = {
    "garage": {
        "file": 'garage.pcd',
        "terrain_assessment_file": 'garage.pcd.dictionary',
        "hyper_start_point": np.array([28.6, -6.7, -10.3]),
        "start_points": {
            0: np.array([-12.59000015,  11.0,         -5.29468489]), 
            1: np.array( [26.05999947, -11.0,         -10.37468719]), 
            2: np.array( [1.59000003, -12.5 ,        -5.66468811]), 
            3: np.array([16.5   ,      8.69999981, -5.3346858 ]), 
            4: np.array([-0.91000003,  4.0     ,    -5.41468811]), 
            5: np.array( [-20.28000069,   4.5 ,        -5.51468706]), 
            6: np.array( [ 17.5 ,       -13.5,        -10.37468719]), 
            7: np.array( [-10.84000015, -20.70000076,  -9.66468811]), 
            8: np.array([ 18.96999931, -11.0,          -5.75468397]), 
            9: np.array( [ 23.05999947, -10.5,        -10.35468674])
        }
    },
    "bridge": {
        "file": 'bridge.pcd',
        "terrain_assessment_file": 'bridge.pcd.dictionary',
        "hyper_start_point": np.array([-53.7, 54.2, -2.7]),
        "start_points": {
            0: np.array([-43.10443115,   3.99802136,   4.46702003]), 
            1: np.array([ 21.61431885, -33.00197983,  -2.77298403]), 
            2: np.array([-34.51068115,  12.49802208,  -4.17298126]), 
            3: np.array([ 15.9268198 , -36.00197983,  -2.6929822 ]), 
            4: np.array([38.98931885, 45.49802399,  1.19701743]), 
            5: np.array([ 3.73931861, 40.74802399,  2.83701849]), 
            6: np.array([ 15.5205698 , -31.50197792,  -2.8729825 ]), 
            7: np.array([-16.44818115, -19.25197792,  -3.58298159]), 
            8: np.array([10.52056885, 42.74802399,  2.46701956]), 
            9: np.array([53.89556885, 35.99802399,  0.33701676])
        }
    },
    "crossing": {
        "file": 'crossing.pcd',
        "terrain_assessment_file": 'crossing.pcd.dictionary',
        "hyper_start_point": np.array([-20.7, 43, -1]),
        "start_points": {
            0: np.array([-7.59375   ,  5.25      , -0.11380386]), 
            1: np.array([44.125     , 13.25      , -4.51380157]), 
            2: np.array([23.625     ,  7.25      , -2.69380188]), 
            3: np.array([-11.40625   ,  15.5       ,  -0.27380371]), 
            4: np.array([-35.15625   , -17.        ,   1.63619614]), 
            5: np.array([ 7.625     , -0.5       , -1.12380219]), 
            6: np.array([34.5       ,  7.25      , -3.56380081]), 
            7: np.array([16.9375    ,  1.5       , -2.02380371]), 
            8: np.array([ 8.96875   , -0.25      , -1.26380157]), 
            9: np.array([-6.65625   , -2.75      ,  0.02619934])
        }
    },
}

ALGORITHMS = {
    "Inward Spiral": {
        "name": "Inward Spiral",
        "hyper_test": "step_param",
        "hyper_time_limit": HYPER_TIME_LIMIT,
        "hyper_min_coverage": HYPER_MIN_COVERAGE,
        "do_experiment": True,
        "experiment_time_limit": EXP_TIME_LIMIT,
        "experiment_results": [],
        "hyper_data": [],
        "formatted_hyper_data": [],
        "cpp": lambda print, motion_planner, cov_points, time_limit, parameters: Spiral(print, motion_planner, PointCloud(print, points= cov_points), time_limit, parameters)
    },
    "BA*": {
        "name": "BA*",
        "hyper_test": "step_param",
        "hyper_time_limit": HYPER_TIME_LIMIT,
        "hyper_min_coverage": HYPER_MIN_COVERAGE,
        "do_experiment": True,
        "experiment_time_limit": EXP_TIME_LIMIT,
        "experiment_results": [],
        "hyper_data": [],
        "formatted_hyper_data": [],
        "cpp": lambda print, motion_planner, cov_points, time_limit, parameters: BAstar(print, motion_planner, PointCloud(print, points= cov_points), time_limit, parameters) ,

    },
    "Sampled BA*": {
        "name": "Sampled BA* & Inward Spiral",
        "hyper_test": "sampled_bastar_param",
        "hyper_time_limit": HYPER_TIME_LIMIT,
        "hyper_min_coverage": HYPER_MIN_COVERAGE,
        "do_experiment": True,
        "experiment_time_limit": EXP_TIME_LIMIT,
        "experiment_results": [],
        "sample_specific_stats": [],
        "hyper_data": [],
        "formatted_hyper_data": [],
        "cpp": lambda print, motion_planner, cov_points, time_limit, parameters: SampledBAstar(print, motion_planner, PointCloud(print, points= cov_points), cost, time_limit, parameters), 
    }
}

