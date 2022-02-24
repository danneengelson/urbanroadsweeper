####################
# Robot parameters #
####################

#Range from left to right
ROBOT_SIZE = 0.75   

#Range from center to left/right
ROBOT_RADIUS = ROBOT_SIZE/2

ROBOT_HEIGHT = 1    

#When moving robot, update points around robot as covered every ROBOT_COVERAGE_STEP_SIZE meter
ROBOT_COVERAGE_STEP_SIZE = 0.2

#Maximum height difference between two points that the robot could manage 
ROBOT_STEP_SIZE = 0.25 * ROBOT_SIZE     

################################ 
#Terrain Assessment parameters #
################################

CELL_SIZE = 0.5
Z_RESOLUTION = 0.1

#20*Z_RESOLUTION for bridge
#7*Z_RESOLUTION for garage
GROUND_OFFSET = 1*Z_RESOLUTION

MIN_FLOOR_HEIGHT = 2
MAX_STEP_HEIGHT = 0.2

#CELL_SIZE**2 * 50 for bridge
#CELL_SIZE**2 * 50 for garage
MIN_POINTS_IN_CELL = CELL_SIZE**2 * 100 #for pointcloud1
#MIN_POINTS_IN_CELL = CELL_SIZE**2 * 50 #for pointcloud2 

#40000 for bridge
#100000 for garage
#15000 for crossing
#FLOOR_LEVEL_HEIGHT_THRESSHOLD = 15000 #for pointcloud1,2,3
#FLOOR_LEVEL_HEIGHT_THRESSHOLD = 5000 #for pointcloud4
FLOOR_LEVEL_HEIGHT_THRESSHOLD = 40000 #for bridge
#MARGIN = 0.25

############################
#Motion Planner Parameters #
############################

STEP_SIZE = 0.1
RRT_STEP_SIZE = 3*STEP_SIZE
ASTAR_STEP_SIZE = 5*STEP_SIZE
UNTRAVERSABLE_THRESHHOLD = 2*STEP_SIZE
RRT_MAX_ITERATIONS = 10000

#######################
#Graph representation #
#######################
GRAPH_DIAG_STEP_SIZE = ROBOT_SIZE

############################
#CPP Algorithms Parameters #
############################

# General
COVEREAGE_EFFICIENCY_GOAL = 1

# Naive RRT CPP
NAIVE_RRT_CPP_MAX_ITERATIONS = RRT_MAX_ITERATIONS
NAIVE_RRT_CPP_GOAL_CHECK_FREQUENCY = 50

# Spiral
#SPIRAL_STEP_SIZE = 0.81549 * ROBOT_SIZE
#SPIRAL_VISITED_TRESHOLD = 0.707 * SPIRAL_STEP_SIZE



calculated = {'step_size': 0.81549, 'visited_threshold': 0.707}
bastar_hypto = {'step_size': 0.872043831037874, 'visited_threshold': 0.6500278902799709}
curved_hypto = {'step_size': 0.9423002094241799, 'visited_threshold': 0.7071654126017254}
spiral_hypto = {'step_size': 0.808161533043964, 'visited_threshold': 0.7357070541013503}

param = calculated
#Bastar
SPIRAL_STEP_SIZE =  spiral_hypto["step_size"] * ROBOT_SIZE
SPIRAL_VISITED_TRESHOLD = spiral_hypto["step_size"] * SPIRAL_STEP_SIZE
BASTAR_STEP_SIZE = bastar_hypto["step_size"] * ROBOT_SIZE
BASTAR_VISITED_TRESHOLD = bastar_hypto["visited_threshold"] * BASTAR_STEP_SIZE
CURVED_BASTAR_STEP_SIZE = curved_hypto["step_size"] * ROBOT_SIZE
CURVED_BASTAR_VISITED_TRESHOLD = curved_hypto["visited_threshold"] * CURVED_BASTAR_STEP_SIZE


#BASTAR_STEP_SIZE = 1.5520893992592577* ROBOT_SIZE
#BASTAR_VISITED_TRESHOLD = 0.2333537281466731 * BASTAR_STEP_SIZE

sampled_hypto = {'coverage_1': 0.8199761400840984, 'max_distance': 1.2312781746957266, 'max_distance_part_II': 4.178941449146873, 'max_iterations': 34.06379599295198, 'min_bastar_coverage': 0.04020596181378239, 'min_spiral_length': 14.843732597409485, 'nbr_of_angles': 7.921301019854004}
sampled_calc = {'coverage_1': 0.87, 'max_distance': 3, 'max_distance_part_II': 5, 'max_iterations': 200, 'min_bastar_coverage': 0.005, 'min_spiral_length': 2, 'nbr_of_angles': 4}
#Random BAstar
RANDOM_BASTAR_VISITED_TRESHOLD = 0.2
RANDOM_BASTAR_MAX_ITERATIONS = sampled_hypto["max_iterations"]
RANDOM_BASTAR_NUMBER_OF_ANGLES = int(round(sampled_hypto["nbr_of_angles"]))
RANDOM_BASTAR_PART_I_COVERAGE = sampled_hypto["coverage_1"]
RANDOM_BASTAR_VARIANT_DISTANCE = sampled_hypto["max_distance"]
RANDOM_BASTAR_VARIANT_DISTANCE_PART_II = sampled_hypto["max_distance_part_II"]
RANDOM_BASTAR_MIN_COVERAGE = 0.01 #sampled_hypto["min_bastar_coverage"]
RANDOM_BASTAR_MIN_SPIRAL_LENGTH = sampled_hypto["min_spiral_length"]