
class Layer:
    ''' Class representing a slice of a point cloud.
    '''
    def __init__(self, layer_bounding_box, full_pcd):
        '''
        Args:
            layer_bounding_box: an Open3D AxisAlignedBoundingBox that defines the bounds of the layer
            full_pcd: Full Point cloud including all existing layers
        '''
        self.points_idx = layer_bounding_box.get_point_indices_within_bounding_box(full_pcd.points)
        self.start_z = layer_bounding_box.min_bound[2]
        self.nbr_of_points = len(self.points_idx)