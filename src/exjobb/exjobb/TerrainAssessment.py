import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import timeit

from exjobb.TA_FloorSegmentation import FloorSegmentation
from exjobb.TA_CellSegmentation import CellSegmentation
from exjobb.TA_CellClassification import CellClassification
from exjobb.TA_PointClassification import PointClassification
from exjobb.Parameters import CELL_SIZE, MAX_STEP_HEIGHT, MIN_FLOOR_HEIGHT, ROBOT_SIZE, Z_RESOLUTION
#From param: GROUND_OFFSET, MIN_POINTS_IN_CELL, FLOOR_LEVEL_HEIGHT_THRESSHOLD
import pickle


class TerrainAssessment():
    """
    A class for doing calculations for terrain assessment to find all
    ground points in a point cloud.
    """

    def __init__(self, print, pcd, param):
        """ 
        Args:
            print: Function for printing messages
            pcd: A PointCloud object with the point cloud that will be analysed.
        """ 
        self.print = print
        self.pcd = pcd.raw_pcd
        self.points = pcd.points
        self.pcd_kdtree = pcd.kdtree
        self.param = param

        self.stats = {}
        
        self.floor_segmentation = FloorSegmentation(print, param)
        self.cell_segmentation = CellSegmentation(print, param)
        self.cell_classification = CellClassification(print, param)
        self.point_classification = PointClassification(print, pcd, param)
        

    def get_classified_points(self):
        """ Analyses point cloud to find all coverable and traversable points in 4 steps:
            1. Floor Segmentation
            2. Cell Segmentation
            3. Cell Classification
            4. Point classification
        Returns:
            Indexes of all coverable and traversable points in the point cloud. 
        """

        potential_coverable_points_idx = np.array([], int)
        uncoverable_border_points = np.empty((0,3))

        

        segmentized_floors, floor_segmentation_stats = self.floor_segmentation.get_segmentized_floors(self.pcd)
        stats = {
            "Floor segmentation": floor_segmentation_stats,
            "Floors": []
        }
        
        for floor in segmentized_floors[0:2]:
            self.print("="*20)
            self.print(floor.name)                
            cell_segmentation_stats = self.cell_segmentation.find_elevation(self.pcd, floor)
            new_coverable_points_idx, new_uncoverable_border_points, cell_classification_stats = self.cell_classification.get_coverable_points_idx(self.pcd, floor)
            potential_coverable_points_idx = np.append(potential_coverable_points_idx, new_coverable_points_idx)
            uncoverable_border_points = np.append(uncoverable_border_points, new_uncoverable_border_points, axis=0)
            stats["Floors"].append({
                "Name": floor.name,
                "Cell segmentation": cell_segmentation_stats,
                "Cell classification": cell_classification_stats
            })

        traversable_points, coverable_points, inaccessible_points, point_classification_stats = self.point_classification.get_classified_points(potential_coverable_points_idx, uncoverable_border_points)
        stats["Point classifciation"] = point_classification_stats
        stats["Parameters"] = {
            "CELL_SIZE": CELL_SIZE,
            "Z_RESOLUTION": Z_RESOLUTION,
            "GROUND_OFFSET": self.param["GROUND_OFFSET"],
            "MIN_FLOOR_HEIGHT": MIN_FLOOR_HEIGHT,
            "MAX_STEP_HEIGHT": MAX_STEP_HEIGHT,
            "MIN_POINTS_IN_CELL": self.param["MIN_POINTS_IN_CELL"],
            "FLOOR_LEVEL_HEIGHT_THRESSHOLD": self.param["FLOOR_LEVEL_HEIGHT_THRESSHOLD"],
            "ROBOT_SIZE": ROBOT_SIZE
        }
        return traversable_points, coverable_points, inaccessible_points, stats
         























































































                    
            
                