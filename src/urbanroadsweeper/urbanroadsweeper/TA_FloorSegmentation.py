
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.signal import argrelextrema
import timeit
from urbanroadsweeper.Floor import Floor
from urbanroadsweeper.Parameters import Z_RESOLUTION, MIN_FLOOR_HEIGHT
#GROUND_OFFSET, FLOOR_LEVEL_HEIGHT_THRESSHOLD

from urbanroadsweeper.Layer import Layer

VISUALIZE = False
SHOW_HISTOGRAM = False 

class FloorSegmentation:
    ''' A class for dividing a point cloud into floors    
    '''

    def __init__(self, print, param):
        ''' 
        Args:
            print: function for printing messages
        '''
        self.print = print
        self.param = param

    def get_segmentized_floors(self, pcd):
        ''' Divides the point cloud into floors. 
        Returns:
            A list of instances of class Floor.
        '''

        start = timeit.default_timer()
        segmentised_floors = []
        current_floor_points_idx = np.array([])

        layers = self.get_layers(pcd)
        
        potential_floor_level_height_layers = self.get_potential_floor_level_height_layers(layers)  
        
        if self.param["ONE_FLOOR"]:
            base_height = potential_floor_level_height_layers[0].start_z
            for layer in layers:
                if layer.start_z < base_height:
                    continue
                current_floor_points_idx = np.append(current_floor_points_idx, layer.points_idx)

            floor = Floor("Floor 1", pcd, current_floor_points_idx)
            segmentised_floors.append(floor)
        else:

            floor_height_layers = self.get_floor_height_layers(potential_floor_level_height_layers)

            base_height = floor_height_layers[0].start_z
            current_floor = 0
            
            top_floor = len(floor_height_layers) - 1

            for layer in layers:
                if layer.start_z < base_height:
                    continue

                if current_floor is not top_floor and layer.start_z >= floor_height_layers[current_floor + 1].start_z:
                    floor = Floor("Floor " + str(current_floor+1), pcd, current_floor_points_idx)
                    segmentised_floors.append(floor)
                    current_floor += 1
                    current_floor_points_idx = np.array([])
                
                current_floor_points_idx = np.append(current_floor_points_idx, layer.points_idx)

            floor = Floor("Floor " + str(top_floor+1), pcd, current_floor_points_idx)
            segmentised_floors.append(floor)

        stats = self.print_result(segmentised_floors, start)

        if VISUALIZE:
            self.visualze_segmentized_floors(segmentised_floors)

        return segmentised_floors, stats

    def get_bounding_box(self, pcd):
        '''Calculates the bounding box of the given point cloud
        Args:
            pcd: Point Cloud
        Returns:
            A dictionary with the min and max boundaries of the point cloud.
        '''
        bounding_box = pcd.get_axis_aligned_bounding_box()
        bounding_box_info = {}
        bounding_box_info["min_x"] = bounding_box.min_bound[0]
        bounding_box_info["min_y"] = bounding_box.min_bound[1]
        bounding_box_info["min_z"] = bounding_box.min_bound[2]
        bounding_box_info["max_x"] = bounding_box.max_bound[0]
        bounding_box_info["max_y"] = bounding_box.max_bound[1]
        bounding_box_info["max_z"] = bounding_box.max_bound[2] 
        return bounding_box_info


    def get_layers(self, pcd):
        ''' Divide the pointcloud along the z-direction into layers. 
        Args:
            pcd: Point cloud to be divided
        Returns:
            A list of instances of class Layer.
        '''
        layers = []
        bounding_box = self.get_bounding_box(pcd)

        layers_start_z = np.arange( bounding_box["min_z"], 
                                    bounding_box["max_z"], 
                                    Z_RESOLUTION)

        for z in layers_start_z:
            layer_bounding_box = o3d.geometry.AxisAlignedBoundingBox(
                [bounding_box["min_x"], bounding_box["min_y"], z],
                [bounding_box["max_x"], bounding_box["max_y"], z + Z_RESOLUTION]
            )
            layer = Layer(layer_bounding_box, pcd)
            layers.append(layer)
            
        return layers

    def get_potential_floor_level_height_layers(self, layers):
        ''' Find layers that has the potential to represent floor level 
        heights.
        By setting the SHOW_HISTOGRAM to True the user can see a histogram
        and adjust the FLOOR_LEVEL_HEIGHT_THRESSHOLD parameter to filter 
        out floor levels. 
        Args:
            values: List of all layers.
        Returns:
            List of layers that could represent a floor level height
        '''

        nbr_of_points_in_layers = np.array([layer.nbr_of_points for layer in layers])
        local_maximas = np.array(argrelextrema(nbr_of_points_in_layers, np.greater))
        potential_maximas = local_maximas[nbr_of_points_in_layers[local_maximas] > self.param["FLOOR_LEVEL_HEIGHT_THRESSHOLD"]] 
        # offset to make sure we dont't miss any points of this floor.
        potential_maximas = potential_maximas - int(self.param["GROUND_OFFSET"] / Z_RESOLUTION)
        self.print("potential_maximas" + str(potential_maximas))
        if SHOW_HISTOGRAM:
            #For tuning FLOOR_LEVEL_HEIGHT_THRESSHOLD:
            plt.plot(np.arange(0, len(nbr_of_points_in_layers))/10, nbr_of_points_in_layers)
            plt.plot(np.arange(0, len(nbr_of_points_in_layers))/10, self.param["FLOOR_LEVEL_HEIGHT_THRESSHOLD"]*np.ones((len(nbr_of_points_in_layers), 1)))
            plt.ylabel('Number of points')
            plt.xlabel('Height [m]')
            plt.title('Number of Points on Different Heights')
            plt.show()

        return [layers[i] for i in potential_maximas]

    def get_floor_height_layers(self, potential_layers): 
        ''' Filters out real floor heights from potential floor heights by 
        making sure there is at least MIN_FLOOR_HEIGHT between the floors.
        Args:
            potential_layers: List of Layers that could be floor level height layers
        Returns
            A list of Layer that represent the floor level heights        
        '''    
        if len(potential_layers) == 1:
            return potential_layers
        
        floor_height_layers = []
        prev_layer = potential_layers[0]
        #potential_ground_layer_idx = potential_ground_layers_idx[0]
        #potential_grounds_z = start_z_of_layers[potential_ground_layer_idx]
        for layer in potential_layers[1:]:
            if layer.start_z - prev_layer.start_z > MIN_FLOOR_HEIGHT:
                floor_height_layers.append(prev_layer)
            prev_layer = layer
        
        #Since the upper floor does not have a roof:
        floor_height_layers.append(layer)
        
        
        return floor_height_layers
    

    def print_result(self, segmentised_floors, start):
        ''' Prints result data of the floor segmentation.
        '''
        end = timeit.default_timer()
        self.print("="*20)
        self.print("FLOOR SEGMENTATION")
        self.print("Number of floors: " + str(len(segmentised_floors)))
        self.print("Computational time: " + str(round(end - start, 1)) + " sec")
        stats = {
            "Number of floors": len(segmentised_floors),
            "Computational time": round(end - start, 1),
            "Floors": []
        }
        for floor, segmentised_floor in enumerate(segmentised_floors):
            self.print("-"*20)
            self.print("Floor " + str(floor + 1))
            self.print("Ground: " + str(np.round(segmentised_floor.z_ground,2)))
            self.print("Ceiling: " + str(np.round(segmentised_floor.z_ceiling,2)))
            self.print("Number of points: " + str(len(segmentised_floor.points_idx_in_full_pcd)))
            stats["Floors"].append({
                "Name": "Floor " + str(floor + 1),
                "Ground": np.round(segmentised_floor.z_ground,2),
                "Ceiling": np.round(segmentised_floor.z_ceiling,2),
                "Number of points": len(segmentised_floor.points_idx_in_full_pcd)
            })
            
        self.print("="*20)

        return stats

    def visualze_segmentized_floors(self, segmentized_floors):
        ''' Visualizes the floors in different colors
        '''
        draw_elements = []
        for nr, floor in enumerate(segmentized_floors):
            pcd = floor.pcd
            pcd.paint_uniform_color(np.array([nr,0,1]))
            draw_elements.append(pcd)

        o3d.visualization.draw_geometries(draw_elements)
