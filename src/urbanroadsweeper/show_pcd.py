import open3d as o3d
import numpy as np
import pickle
from show_config import PCD_DATA
from urbanroadsweeper.ROSMessage import RED, GREEN, BLUE, LIGHTBLUE, YELLOW, TRAVERSABLE, COVERABLE, PATH
from urbanroadsweeper.Environments import PointCloudEnvironment
import sys

def main():

    environment_name = sys.argv[1]
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

    all_geometries = [full_pcd, coverable_pcd, traversable_pcd]
    o3d.visualization.draw_geometries(all_geometries,
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

if __name__ == '__main__':
    main()