import pickle
from exjobb.PointCloud import PointCloud
from exjobb.TerrainAssessment import TerrainAssessment
from exjobb.Environment_Parameters import environment_parameters

class Environment():
    def __init__(self, print, cache_dictionary, do_terrain_assessment=False):
        self.cache_dictionary = cache_dictionary
        self.print = print
        if do_terrain_assessment:
            self.print("Doing terrain assessment.")
            traversable_points, coverable_points, inaccessible_points = self.do_terrain_assessment()
            self.print("Saved terrain assessment in " + str(self.cache_dictionary))
        else:
            traversable_points, coverable_points, inaccessible_points = self.get_terrain_assessment_data()
        
        self.traversable_pcd = PointCloud(print, points= traversable_points)
        self.coverable_pcd = PointCloud(print, points= coverable_points)
        self.inaccessible_pcd = PointCloud(print, points= inaccessible_points)

    def get_terrain_assessment_data(self):
        try:
            with open(self.cache_dictionary, 'rb') as cached_pcd_file:
                cache_data = pickle.load(cached_pcd_file)
                coverable_points = cache_data.get("coverable_points")
                traversable_points = cache_data.get("traversable_points")
                inaccessible_points = cache_data.get("inaccessible_points")
        except:
            self.print("Cache file not accessible. Doing terrain assessment.")
            traversable_points, coverable_points, inaccessible_points = self.do_terrain_assessment()
            self.print("Saved terrain assessment in " + str(self.cache_dictionary))

        return traversable_points, coverable_points, inaccessible_points

class PointCloudEnvironment(Environment):
    def __init__(self, print, cache_dictionary, pcd_file, do_terrain_assessment=False):
        self.full_pcd = PointCloud(print, file=pcd_file)
        self.pcd_file = pcd_file
        super().__init__(print, cache_dictionary, do_terrain_assessment)

    def do_terrain_assessment(self):
        '''Calculating points which are theoretically possible to cover, iignoring
        the size of the robot.
        Returns:
            Coverable points and their indices in the point cloud as NumPy arrays.
        '''
        filename = self.pcd_file.split("/")[-1]
        if environment_parameters.get(filename):
            parameters = environment_parameters[filename]
        else: 
            print("ERROR: No environment parameters for " + filename)
            return 

        

        terrain_assessment = TerrainAssessment(self.print, self.full_pcd, parameters)
        traversable_points, coverable_points, inaccessible_points, stats = terrain_assessment.get_classified_points()

        with open(self.cache_dictionary, 'wb') as cached_pcd_file:
            cache_data = {
                "coverable_points": coverable_points, 
                "traversable_points": traversable_points,
                "inaccessible_points": inaccessible_points,
                "stats": stats
                }
            pickle.dump(cache_data, cached_pcd_file)        
        return traversable_points, coverable_points, inaccessible_points