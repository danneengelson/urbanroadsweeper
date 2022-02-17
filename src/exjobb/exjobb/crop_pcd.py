#import open3d as o3d
#import numpy as np
import sys

def main():
    pcd = sys.argv[1] + sys.argv[2]
    center_point = [float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]
    area_radius = sys.argv[6] 
    filename = sys.argv[7]
    print("="*20)
    print("Point cloud file: " + str(pcd))
    print("Center_point: " + str(center_point))
    print("Area radius: " + str(area_radius))
    print("New filename: " + str(filename))
    #crop_pcd(pcd, center_point, area_radius, filename)
    print("="*20)


def crop_pcd(pcd, center_point, area_radius, filename):
    print("Reading point cloud file...")
    pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/exjobb/exjobb/" + pcd)
    print(pcd + " consists of " + str(len(pcd.points)) + " points.")
    print("Cropping point cloud...")
    
    all_points = np.asarray(pcd.points)
    all_points = all_points - np.array(center_point)    
    all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < area_radius]
    
    print(all_points)

    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(all_points)
    o3d.io.write_point_cloud(filename, new_pcd)
    print("Done cropping")

if __name__ == '__main__':
    main()