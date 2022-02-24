
RESULTS = {
    "opt_param_file": "results_opt_param.csv",
    "opt_stats_file": "results_opt_stats.csv",
}

PCD_DATA = {
    "garage": {
        "name": "Garage",
        "spiral": 'garage.pcd_spiral.dictionary',
        "bastar": 'garage.pcd_bastar.dictionary',
        "sampled": 'garage.pcd_sampled.dictionary'
    },
    "bridge": {
        "name": "Bridge",
        "spiral": 'bridge.pcd_spiral.dictionary',
        "bastar": 'bridge.pcd_bastar.dictionary',
        "sampled": 'bridge.pcd_sampled.dictionary'
    },
    "crossing": {
        "name": "Crossing",
        "spiral": 'crossing.pcd_spiral.dictionary',
        "bastar": 'crossing.pcd_bastar.dictionary',
        "sampled": 'crossing.pcd_sampled.dictionary'
    }
}

ALGORITHM_DATA = {
    "bastar": {
        "name": "BA*",
        'line': 'r',
        'confidence_color': (1.0, 0.0, 0.0, 0.3)
    },
    "spiral": {
        "name": "Inward Spiral",
        'line': 'b',
        'confidence_color': (0.0, 0.0, 1.0, 0.3)
    },
    "sampled": {
        "name": "Sampled BA* & Inward Spiral",
        'line': 'g',
        'confidence_color': (0.0, 1.0, 0.0, 0.3)
    }
}