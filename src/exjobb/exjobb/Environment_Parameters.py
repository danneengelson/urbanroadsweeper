
import numpy as np
#Environment specific tuned parameters

environment_parameters = {
    "garage.pcd": {
        "GROUND_OFFSET": 0.75,
        "MIN_POINTS_IN_CELL": 12.5,
        "FLOOR_LEVEL_HEIGHT_THRESSHOLD": 100000,
        "ONE_FLOOR": False,
        "GET_SECOND_BIGGEST_ISLAND": False,
        "FALSE_TRAVERSABLE_POINTS": np.array([
            [24.395610809326172, 12.705216407775879, -5.311060428619385],
            [-17.590679168701172, -3.7045161724090576, -6.118121147155762]
        ]),
    },  
    "bridge.pcd": {
        "GROUND_OFFSET": 2.3,
        "MIN_POINTS_IN_CELL": 9,
        "FLOOR_LEVEL_HEIGHT_THRESSHOLD": 40000,
        "ONE_FLOOR": False,
        "GET_SECOND_BIGGEST_ISLAND": True,
        "FALSE_TRAVERSABLE_POINTS": np.array([
            [-98.5624, 182.8, -30.83],
            [ 0.8125    ,  93.30000305, -32.33319855],
            [-17.590679168701172, -3.7045161724090576, -6.118121147155762],
        ]),
    },
    "crossing.pcd": {
        "GROUND_OFFSET": 0.1,
        "MIN_POINTS_IN_CELL": 25,
        "FLOOR_LEVEL_HEIGHT_THRESSHOLD": 15000,
        "ONE_FLOOR": True,
        "GET_SECOND_BIGGEST_ISLAND": False,
        "FALSE_TRAVERSABLE_POINTS": np.array([
            [-4.5,         1.5 ,       -0.18380356],
        ]),
    }
}
    