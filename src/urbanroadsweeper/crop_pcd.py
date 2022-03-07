import open3d as o3d
import numpy as np
import sys
import os

def main():
    pcd_file = sys.argv[1] + sys.argv[2]
    center_point = [float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]
    area_radius = float(sys.argv[6]) 
    filename = sys.argv[7]
    print("="*20)
    print("Point cloud file: " + str(pcd_file))
    print("Center_point: " + str(center_point))
    print("Area radius: " + str(area_radius))
    print("New filename: " + str(filename))
    special = None
    if len(sys.argv) > 8:
        special = sys.argv[8]
        print("Special cropping name: " + str(special))
    crop_pcd(pcd_file, center_point, area_radius, filename, special)
    print("="*20)


def crop_pcd(pcd_file, center_point, area_radius, filename, special):
    print("Reading point cloud file...")
    #pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd)
    pcd = o3d.io.read_point_cloud(pcd_file)
    print(str(pcd_file) + " consists of " + str(len(pcd.points)) + " points.")
    print("Cropping point cloud...")
    
    all_points = np.asarray(pcd.points)
    all_points = all_points - np.array(center_point)    
    all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < area_radius]

    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(all_points)

    if special == "bridge":
        cell_bounding_box = o3d.geometry.AxisAlignedBoundingBox(
                [-108.32318115, -51.00197983, -100],
                [76.61431885, 54.74802399, 100]
            )
        new_pcd = new_pcd.crop(cell_bounding_box)   
        all_points = np.asarray(new_pcd.points)
        all_points= all_points[~(np.logical_and(all_points[:,0] < -45, all_points[:,1] < -1))]         
        new_pcd.points = o3d.utility.Vector3dVector(all_points)

    print("Cropped Point cloud " + str(filename) + " consists of " + str(len(new_pcd.points)) + " points.")
    o3d.io.write_point_cloud(filename, new_pcd)
    print("Done cropping")

if __name__ == '__main__':
    main()