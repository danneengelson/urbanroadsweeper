
import open3d as o3d
import numpy as np
import os

from urbanroadsweeper.Parameters import ROBOT_RADIUS, ROBOT_COVERAGE_STEP_SIZE

class PointCloud:
    """
    A class for doing calculations on a point cloud
    and keeping track of covered points and coverage efficiency.
    """

    def __init__(self, print, file = None, points = None, new_point_cloud = False ):
        """ 
        Args:
            print: Function for printing messages
            file: A .pcd file that defines the points in a point cloud
            points: A Nx3 numpy array with all points in the point cloud 
        """ 

        self.print = print

        if file is not None:
            self.print("Reading point cloud file...")
            
            if new_point_cloud:
                self.filter_pointcloud_official(file)
                #self.split_point_cloud_file(file)
                #return


            #self.raw_pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file)
            self.raw_pcd = o3d.io.read_point_cloud(file)
            self.print(self.raw_pcd)
            self.points = np.asarray(self.raw_pcd.points)
            #mean = np.mean(self.points, axis=0)
            #self.print(mean)
            #self.points = self.points - mean
        elif points is not None:
            self.raw_pcd = o3d.geometry.PointCloud()
            self.raw_pcd.points = o3d.utility.Vector3dVector(points)
            self.points = points
        else:
            self.print("Missing pointcloud argument")
            return


        
        self.kdtree = o3d.geometry.KDTreeFlann(self.raw_pcd)
        self.covered_points_idx =  np.array([])


    def visit_position(self, position, apply_unique=False):
        """ Mark points around position as covered
        Args:
            position: The position to visit
            apply_unique: If True, make sure covered_points_idx doesn't have duplicates.
                          Used when function not called from self.visit_path_to_position.
        """
        [k, idx, _] = self.kdtree.search_radius_vector_3d(position, ROBOT_RADIUS)
        self.covered_points_idx = np.append(self.covered_points_idx, idx)
        
        if apply_unique:
            self.covered_points_idx = np.unique(self.covered_points_idx)

    def visit_path_to_position(self, goal_pos, start_pos):
        """ Go in a straight line to a position and mark the points along the path as covered
        Args:
            goal_pos: The position to visit
            start_pos: current position
        """
        if np.linalg.norm( goal_pos - start_pos ) > ROBOT_COVERAGE_STEP_SIZE:
            steps = int(np.ceil( np.linalg.norm( goal_pos - start_pos) / ROBOT_COVERAGE_STEP_SIZE ))
            path_to_pos = np.linspace(start_pos, goal_pos, steps)
        else:
            path_to_pos = [goal_pos]

        for step in path_to_pos:
            self.visit_position(step)

        self.covered_points_idx = np.unique(self.covered_points_idx)

    def visit_path(self, path):
        """ Go throught the positions in the path and mark the points along the path as covered
        Args:
            path: Positions in the path
        """
        
        prev_pos = path[0]
        for pos in path[1:]:
            self.visit_path_to_position(pos, prev_pos)
            prev_pos = pos

    def unvisit_position(self, position):
        """ Mark points around position as uncovered
        Args:
            position: The position to unvisit
        """
        [k, idx, _] = self.kdtree.search_radius_vector_3d(position, ROBOT_RADIUS) 
        self.covered_points_idx = self.covered_points_idx[ np.isin(self.covered_points_idx, idx, assume_unique=True, invert=True) ]

    #SKA EVENTUELLT BORT
    def get_visiting_rate_in_area(self, point, radius):
        [k, idx, _] = self.kdtree.search_radius_vector_3d(point, radius)
        total_points = len(idx)
        nbr_of_visited = len(np.intersect1d(self.covered_points_idx, np.asarray(idx)))
        return nbr_of_visited/total_points


    def get_covered_points_as_pcd(self):
        """ Returns a point cloud message with only covered points
        Returns:
            A publishable point cloud message
        """
        return self.point_cloud(self.points[self.covered_points_idx.astype(int), :], 'my_frame')

    def get_coverage_efficiency(self):
        """ Returns the percentage of the point cloud that has been covered
        Returns:
            Coverage efficiency between 0 and 1
        """

        if len(self.covered_points_idx) == 0:
            return 0
            
        #self.covered_points_idx = np.unique(self.covered_points_idx, axis=0)
        return len(self.covered_points_idx) / len(self.points)

    def get_coverage_count_per_point(self, path):
        """ Returns how many times every point has been covered on average
        Returns:
            Coverage count per point on average
        """
        if not len(path):
            return 0

        covered_points_idx = np.array([]).astype(np.int32)
        prev_pos = path[0]
        for pos in path[1:]:
            if np.array_equal(pos, prev_pos):
                continue 

            if np.linalg.norm( pos - prev_pos ) > ROBOT_COVERAGE_STEP_SIZE:
                steps = int(np.ceil( np.linalg.norm( pos - prev_pos) / ROBOT_COVERAGE_STEP_SIZE ))
                path_to_pos = np.linspace(prev_pos, pos, steps)
            else:
                path_to_pos = [pos]

            for step in path_to_pos:
                [k, idx, _] = self.kdtree.search_radius_vector_3d(step, ROBOT_RADIUS)
                covered_points_idx = np.append(covered_points_idx, idx)


            prev_pos = pos
        
        nodes, inv, counts = np.unique(covered_points_idx, return_inverse=True, return_counts=True)
        return np.mean(counts)

    def point_cloud(self, points, parent_frame):
        import sensor_msgs.msg as sensor_msgs
        import std_msgs.msg as std_msgs

        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.
        
        if not len(points):
            return sensor_msgs.PointCloud2()

        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data,
        )

    def find_k_nearest(self, position, k):
        """ Returns the K nearest points
        Args:
            position: Arbitary position 
            k: Number of points to return
        Returns:
            K nearest points from position
        """
        [k, idx, _] = self.kdtree.search_knn_vector_3d(position, k)
        return self.points[idx, :]

    def distance_to_nearest(self, position):
        """ Returns the distance to closest point in point cloud 
        to given position.
        Args:
            position: Arbitary position 
        Returns:
            Distance in meters
        """
        nearest_point = self.find_k_nearest(position, 1)
        return np.linalg.norm(position - nearest_point[0])

    def points_idx_in_radius(self, position, radius):
        """ Returns indexes of points in point cloud within given radius 
        from given position.
        Args:
            position: Arbitary position 
            radius: Search radius
        Returns:
            Array of indexes of points
        """
        [k, idx, _] = self.kdtree.search_radius_vector_3d(position, radius)
        return np.asarray(idx, dtype=int)
    

    #Following code is used to get parts of big point clouds

    def filter_pointcloud(self):
        x1 = 351463
        x2 = 351517
        y1 = 4022867
        y2 = 4022914
        z0 = 58

        pcd_path = "out.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path, print_progress=True)
        
        self.print("Filtering point cloud...")

        origin = np.array([[x1,y1,z0]]) 
        self.points = np.array([[0,0,0]]) 

        all_points = np.asarray(pcd.points)
        
        c = np.zeros((1,2))
        c[0,0] = (x2-x1)/2 + x1
        c[0,1] = (y2-y1)/2 + y1
        temp = all_points[:,:2] - np.repeat(c, all_points.shape[0], axis=0)
        valid_idx = np.where(np.linalg.norm(temp, axis=1) < 30)
        self.points = all_points[valid_idx] - np.array([c[0,0], c[0,1], z0])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/pointcloud.pcd", pcd)

    def filter_pointcloud3(self, file):
        x_center = 318274.875 
        y_center =  4156388.7
        z_center = 53
        x_center = 0 
        y_center =  0
        z_center = 0
        pcd_path = "bridge_2.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print(pcd_path + " consists of " + str(len(pcd.points)) + " points.")
        self.print("Filtering point cloud...")

        

        all_points = np.asarray(pcd.points)

        random_idx = np.random.choice(len(all_points), 1, replace=False)[0]
        #x_center = all_points[random_idx,0]
        #y_center = all_points[random_idx,1]



        self.print("Chosen point: " + str((x_center, y_center)))
        
        c = np.zeros((1,2))
        #c[0,0] = (x2-x1)/2 + x1
        #c[0,1] = (y2-y1)/2 + y1
        c[0,0] = x_center
        c[0,1] =  y_center
        temp = all_points[:,:2] - np.repeat(c, all_points.shape[0], axis=0)
        valid_idx = np.where(np.linalg.norm(temp, axis=1) < 250)
        #self.points = all_points[valid_idx] - np.array([c[0,0], c[0,1], z_center])
        #self.points = self.points[self.points[:,0] > -140]
        all_points = all_points[all_points[:,0] < 15]
        all_points = all_points[all_points[:,1] < 10]
        all_points = all_points[all_points[:,2] < -2]
        #all_points= all_points[~(np.logical_and(all_points[:,0] < -40, all_points[:,1] < 0))]
        self.print(len(all_points))
        mean = np.mean(all_points, axis=0)
        self.print(mean)

        all_points = all_points - mean      
        self.print(len(all_points))


        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file, pcd)
        self.print("Done filtering")

    def split_point_cloud_file(self, file):
        
        pcd_path = file
        #pcd_path = "urban02.pcd"
        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print("Splitting point cloud...")
        
        all_points = np.asarray(pcd.points)

        center_point = np.mean(all_points, axis=0)
        print(center_point)
        #center_point = np.array([0,0,0])

        right = all_points[all_points[:,0] > center_point[0]]
        left = all_points[all_points[:,0] <= center_point[0]]
        right_up = right[right[:,1] > center_point[1]]
        right_down = right[right[:,1] <= center_point[1]]
        left_up = left[left[:,1] > center_point[1]]
        left_down = left[left[:,1] <= center_point[1]]
        self.print("Done splitting points...")

        for idx, part in enumerate([right_up, right_down, left_up, left_down]):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(part)
            o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file[0:-4] + "_" + str(idx) + ".pcd", pcd)
            self.print(pcd)
            self.print("Done with " + str(idx+1) + "/4")




    def filter_pointcloud2(self):
        x1 = 0
        x2 = 10
        y1 = 10
        y2 = 20
        z0 = 0

        pcd_path = os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/pointcloud.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(pcd_path, print_progress=True)
        self.print("Filtering point cloud...")

        origin = np.array([[x1,y1,z0]]) 
        self.points = np.array([[0,0,0]]) 

        all_points = np.asarray(pcd.points)
        
        c = np.zeros((1,2))
        c[0,0] = (x2-x1)/2 + x1
        c[0,1] = (y2-y1)/2 + y1
        temp = all_points[:,:2] - np.repeat(c, all_points.shape[0], axis=0)
        valid_idx = np.where(np.linalg.norm(temp, axis=1) < 5)
        self.points = all_points[valid_idx] - np.array([c[0,0], c[0,1], z0])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/smallpointcloud.pcd", pcd)

    def filter_pointcloud_official(self, file):
        x_center = 351490
        y_center =  4022890.5
        z_center = 58
        pcd_path = "urban02_0.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print(pcd_path + " consists of " + str(len(pcd.points)) + " points.")
        self.print("Filtering point cloud...")

        
        all_points = np.asarray(pcd.points)

        random_idx = np.random.choice(len(all_points), 1, replace=False)[0]
        x_center = all_points[random_idx,0]
        y_center = all_points[random_idx,1]

        total_center = np.array([x_center, y_center, z_center])
        all_points = all_points - total_center

        self.print(all_points)
        self.print("Chosen point: " + str((x_center, y_center)))
        
        all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < 50]
        
        self.print(all_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file, pcd)
        self.print("Done filtering")
    


############################################################################


    # THIS IS HOW GARAGE WAS FOUND:
    def filter_pointcloud_official_garage(self, file):
        x_center = 351490
        y_center =  4022890.5
        z_center = 58
        pcd_path = "urban05.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print(pcd_path + " consists of " + str(len(pcd.points)) + " points.")
        self.print("Filtering point cloud...")
        

        all_points = np.asarray(pcd.points)

        random_idx = np.random.choice(len(all_points), 1, replace=False)[0]

        total_center = np.array([x_center, y_center, z_center])
        all_points = all_points - total_center

        self.print(all_points)
        self.print("Chosen point: " + str((x_center, y_center)))
        
        all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < 30]
        
        self.print(all_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file, pcd)
        self.print("Done filtering")

    # THIS IS A GUESS OF HOW CROSSING WAS FOUND:
    def filter_pointcloud_official_garage(self, file):
        x_center = 326608.680
        y_center =  4152387.95
        z_center = 48.8488709
        area_radius = 50
        pcd_path = "urban17.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print(pcd_path + " consists of " + str(len(pcd.points)) + " points.")
        self.print("Filtering point cloud...")
        

        all_points = np.asarray(pcd.points)

        random_idx = np.random.choice(len(all_points), 1, replace=False)[0]

        total_center = np.array([x_center, y_center, z_center])
        all_points = all_points - total_center

        self.print(all_points)
        self.print("Chosen point: " + str((x_center, y_center)))
        
        all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < area_radius]
        
        self.print(all_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file, pcd)
        self.print("Done filtering")


    # THIS IS A WILD GUESS OF HOW BRIDGE WAS FOUND BEFORE CUTTING EDGES...
    def filter_pointcloud_official_bridge(self, file):
        x_center = 318274.875 
        y_center =  4156388.7
        z_center = 53
        area_radius = 250
        pcd_path = "urban02.pcd"

        self.print("Reading point cloud file...")
        pcd = o3d.io.read_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + pcd_path)
        self.print(pcd_path + " consists of " + str(len(pcd.points)) + " points.")
        self.print("Filtering point cloud...")
        

        all_points = np.asarray(pcd.points)

        random_idx = np.random.choice(len(all_points), 1, replace=False)[0]

        total_center = np.array([x_center, y_center, z_center])
        all_points = all_points - total_center

        self.print(all_points)
        self.print("Chosen point: " + str((x_center, y_center)))
        
        all_points = all_points[ np.linalg.norm(all_points[:,0:2], axis=1) < area_radius]
        
        self.print(all_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        o3d.io.write_point_cloud(os.getcwd() + "/src/urbanroadsweeper/urbanroadsweeper/" + file, pcd)
        self.print("Done filtering")