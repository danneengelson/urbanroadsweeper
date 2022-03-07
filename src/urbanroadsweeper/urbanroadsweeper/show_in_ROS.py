
from copy import deepcopy
from urbanroadsweeper.BAstar import BAstar
from urbanroadsweeper.Environments import MapEnvironment, PointCloudEnvironment
from urbanroadsweeper.MotionPlanner import MotionPlanner
from urbanroadsweeper.PointCloud import PointCloud
import urbanroadsweeper.ROSMessage as ROSMessage
from urbanroadsweeper.SampledBAstar import SampledBAstar
from urbanroadsweeper.Spiral import Spiral
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import visualization_msgs.msg as visualization_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import numpy as np
import pickle
import open3d as o3d
import time
from urbanroadsweeper.show_config import PCD_DATA
from urbanroadsweeper.ROSMessage import RED, GREEN, BLUE

#==============================
# THIS IS INCOMPLETE CODE
#==============================

class MainNode(Node):

    def __init__(self):
        super().__init__('MainNode')

        # SET PCD=parameter from launch file "show.launch.py" which gets it from input when doing show.sh
        environment_name = "garage" #hardcoded

        #Start positions publisher
        self.markers_pub = self.create_publisher(visualization_msgs.MarkerArray, 'start_positions', 3000)
        
        #Terrain Assessment publishers
        self.pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'pcd', 10)
        self.coverable_pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'coverable_pcd', 100)
        self.traversable_pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'traversable_pcd', 100)
        self.inaccessible_pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'inaccessible_pcd', 100)
        
        #Path publishers
        self.bastar_path_pub = self.create_publisher(nav_msgs.Path, 'bastar_path', 10)
        self.spiral_path_pub = self.create_publisher(nav_msgs.Path, 'spiral_path', 10)
        self.sampled_path_pub = self.create_publisher(nav_msgs.Path, 'sampled_path', 10)

        #Visited points publisher
        self.visited_pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'visited_pcd', 100)
        self.visited_ground_pcd_pub = self.create_publisher(sensor_msgs.PointCloud2, 'visited_ground_pcd', 100)

        with open(PCD_DATA[environment_name]["results_file"], 'rb') as cached_pcd_file:
            results = pickle.load(cached_pcd_file)  

        #Publish start positions
        hyper_start_position = results["environment_setup"]["hyper_start_point"]
        start_positions = results["environment_setup"]["start_points"]
        
        self.markers_msg = visualization_msgs.MarkerArray()
        stamp = self.get_clock().now().to_msg()
        msg = ROSMessage.point_marker(0, stamp, hyper_start_position, GREEN, "hyper_start_position")
        self.markers_msg.markers.append(msg)
        
        for idx, point in enumerate(start_positions):
            stamp = self.get_clock().now().to_msg()
            msg = ROSMessage.point_marker(idx+1, stamp, point, BLUE, str(idx))
            self.markers_msg.markers.append(msg)

        self.markers_pub.publish(self.markers_msg)

        #Publish Terrain Assessment
        environment = PointCloudEnvironment(self.print, PCD_DATA[environment_name]["terrain_assessment_file"], PCD_DATA[environment_name]["pcd_file"], False)
        full_pcd  = environment.full_pcd
        traversable_pcd = environment.traversable_pcd
        coverable_pcd = environment.coverable_pcd
        inaccessible_pcd = environment.inaccessible_pcd        
            
        self.pcd_pub.publish(full_pcd.point_cloud(full_pcd.points, 'my_frame'))
        self.coverable_pcd_pub.publish(coverable_pcd.point_cloud(coverable_pcd.points, 'my_frame'))
        self.traversable_pcd_pub.publish(traversable_pcd.point_cloud(traversable_pcd.points, 'my_frame'))
        self.inaccessible_pcd_pub.publish(inaccessible_pcd.point_cloud(inaccessible_pcd.points, 'my_frame'))
        
        #Publish Paths
        bastar

    def print(self, object_to_print):
        self.get_logger().info(str(object_to_print))

def main(args=None):
    rclpy.init(args=args)
    ros_node = MainNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
