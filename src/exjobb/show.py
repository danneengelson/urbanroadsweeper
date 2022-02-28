import open3d as o3d
import numpy as np
import pickle
from show_config import PCD_DATA
from exjobb.ROSMessage import RED, GREEN, BLUE, LIGHTBLUE, YELLOW, TRAVERSABLE, COVERABLE, PATH
from exjobb.Environments import PointCloudEnvironment


import sys

def main():

    environment_name = sys.argv[1]
    algorithm_name = sys.argv[2]

    environment = PointCloudEnvironment(print, PCD_DATA[environment_name]["terrain_assessment_file"], PCD_DATA[environment_name]["pcd_file"], False)

    full_pcd = o3d.geometry.PointCloud()
    full_pcd.points = o3d.utility.Vector3dVector(environment.full_pcd.points)
    #full_pcd.paint_uniform_color([0.1, 0.1, 0.1])

    traversable_pcd = o3d.geometry.PointCloud()
    traversable_pcd.points = o3d.utility.Vector3dVector(environment.traversable_pcd.points +  np.array([0,0,0.1]))
    traversable_pcd.paint_uniform_color(TRAVERSABLE)

    coverable_pcd = o3d.geometry.PointCloud()
    coverable_pcd.points = o3d.utility.Vector3dVector(environment.coverable_pcd.points)
    coverable_pcd.paint_uniform_color(COVERABLE)

    results_file = PCD_DATA[environment_name][algorithm_name]
    with open(results_file, 'rb') as cached_pcd_file:
        results = pickle.load(cached_pcd_file)    

    if results:
        # Show best path generated during hyper optimization
        path_length = len(results["best_hyper_path"])-1
        lines = np.append([np.arange(path_length)], [np.arange(path_length) + 1], axis=0).transpose() 
        path = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(results["best_hyper_path"]),
            lines=o3d.utility.Vector2iVector(lines),
        )
        path.paint_uniform_color(PATH)
        path.translate(np.asarray([0,0,0.2]))

        # Show hyper start point
        hyper_start_point = results["setup"]["environment"]["hyper_start_point"]
        hyper_start_point_box = o3d.geometry.TriangleMesh.create_box(width=0.5,
                                                        height=0.5,
                                                        depth=0.5)
        hyper_start_point_box.translate(hyper_start_point)
        hyper_start_point_box.paint_uniform_color(GREEN)


        # Show start points in Experiment 2
        nbr_of_start_points = results["setup"]["nbr_of_start_points"]
        start_points = results["setup"]["environment"]["start_points"]
        print(start_points)
        start_point_boxes = []
        for idx in np.arange(nbr_of_start_points):
            start_point_box = o3d.geometry.TriangleMesh.create_box( width=0.5,
                                                                    height=0.5,
                                                                    depth=0.5)
            start_point_box.translate(start_points[idx])
            start_point_box.paint_uniform_color(BLUE)
            start_point_boxes.append(start_point_box)

    all_geometries = [full_pcd, coverable_pcd, traversable_pcd, path, hyper_start_point_box]
    all_geometries.extend(start_point_boxes)
    o3d.visualization.draw_geometries(all_geometries,
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

    

if __name__ == '__main__':
    main()