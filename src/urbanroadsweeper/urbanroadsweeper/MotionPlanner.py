import numpy as np
import networkx as nx

from heapq import heappush, heappop
from itertools import count

from urbanroadsweeper.Tree import Tree
from urbanroadsweeper.Parameters import STEP_SIZE, RRT_STEP_SIZE, ASTAR_STEP_SIZE, UNTRAVERSABLE_THRESHHOLD, RRT_MAX_ITERATIONS

TRAPPED = 0
ADVANCED = 1
REACHED = 2

class MotionPlanner():
    ''' Class for handling of the motion planning of the robot. Planning the shortest
    obstacle free paths to given points and checks if a step is possible.
    '''

    def __init__(self, print, traversable_pcd):
        '''
        Args:
            print: function for printing messages
            traversable_pcd: Point Cloud with only traversable points.
        '''
        self.print = print
        self.traversable_pcd = traversable_pcd
        self.traversable_points = np.asarray(traversable_pcd.points)

    def RRT(self, start_point, end_point):
        ''' Calculates the shortest path between two points using RRT
        Args:
            start_point: [x,y,z] NumPy array with position of the start point  
            end_point: [x,y,z] NumPy array with position of the end point  
        Returns:
            The path as a Nx3 NumPy array with points 
        '''
        tree_a = Tree(name="start")
        tree_a.add_node(start_point)
        tree_b = Tree(name="goal")
        tree_b.add_node(end_point)

        nbr_of_points_in_pcd = len(self.traversable_pcd.points)

        def get_random_point():
            return self.traversable_pcd.points[np.random.randint(nbr_of_points_in_pcd)]

        for i in range(RRT_MAX_ITERATIONS):
            random_point = get_random_point()
            new_point_1, status = self.extend(tree_a, random_point)

            if status != TRAPPED:
                new_point_2, status = self.extend(tree_b, new_point_1)
                if status == REACHED:
                    path = self.get_shortest_path(tree_a, tree_b, new_point_2)
                    return path
            
            tree_a, tree_b = tree_b, tree_a
        
        self.print("Failed to find path using RRT")
        return False

    def extend(self, tree, extension_point):
        ''' Creates a new node in a given tree, which is placed on a specific distance
        towards a given point from the closest node in the tree.
        Args:
            tree: Tree that will be extended
            extension_point: point towards which the tree will be extended.
        Returns
            The position and status of the new node.
        '''
        nearest_node_idx, nearest_point = tree.nearest_node(extension_point)
        new_point = self.new_point_towards(nearest_point, extension_point, RRT_STEP_SIZE)

        if self.is_valid_step(nearest_point, new_point):
            distance = np.linalg.norm(new_point - nearest_point)
            new_node_idx = tree.add_node(new_point)
            tree.add_edge(nearest_node_idx, new_node_idx, distance)
            if np.array_equal(new_point, extension_point):
                return new_point, REACHED
            else:
                return new_point, ADVANCED
        else:
            return new_point, TRAPPED


    def get_shortest_path(self, tree_1, tree_2, connection_point):
        ''' Given two trees and a connection point, find the shortest path
        from the first node in the forst tree to the last in the second
        Args:
            tree_1: Tree with starting point at node index 0
            tree_2: Tree with ending point as the last node.
            connection_point: The point that connects the two trees
        Returns:
            The path as a Nx3 NumPy array with points 
        '''
        full_tree = nx.disjoint_union(tree_1.tree, tree_2.tree)
        full_nodes = np.append(tree_1.nodes, tree_2.nodes, axis=0)
        connection_idx_tree_1 = len(tree_1.nodes) - 1
        connection_idx_tree_2 = tree_2.nearest_node(connection_point)[0]
        connection_idx_tree_2_full_tree = connection_idx_tree_1 + connection_idx_tree_2 + 1        
        full_tree.add_edge(connection_idx_tree_1, connection_idx_tree_2_full_tree, weight=0)

        start_idx = 0
        end_idx = len(tree_1.nodes)

        def dist(a, b):
            a_point = full_nodes[a]
            b_point = full_nodes[b]
            return np.linalg.norm(b_point - a_point)

        path = nx.astar_path(full_tree, start_idx, end_idx, heuristic=dist)
        return np.array([ full_nodes[idx] for idx in path ])
            

    def Astar(self, start_point, end_point):
        ''' Calculates the shortest path between two points using Astar and smoothes the 
        path with AStarSPT. If it fails, it tries to create the path with RRT.
        Args:
            start_point: [x,y,z] NumPy array with position of the start point  
            end_point: [x,y,z] NumPy array with position of the end point 
        Returns:
            The path as a Nx3 NumPy array with points 
        Reference:
            Code partly taken from NetworkX:
             https://networkx.org/documentation/stable/_modules/networkx/algorithms/shortest_paths/astar.html
        '''
        
        self.astar_points = np.array([start_point, end_point])

        if self.is_valid_step(start_point, end_point) or np.array_equal(start_point, end_point):
            return self.astar_points
        
        start = 0
        target = 1

        def distance(a, b):
            a_point = self.astar_points[a]
            b_point = self.astar_points[b]
            return np.linalg.norm(a_point - b_point)

        push = heappush
        pop = heappop

        c = count()
        queue = [(0, next(c), start, 0, None)]

        enqueued = {}
        explored = {}

        while queue:
            # Pop the smallest item from queue.
            _, __, curnode, dist, parent = pop(queue)

            if curnode == target:
                path = [curnode]
                node = parent
                while node is not None:
                    path.append(node)
                    node = explored[node]
                path.reverse()
                astar_spt_path = self.AstarSPT(self.astar_points[path])
                return astar_spt_path

            if curnode in explored:
                # Do not override the parent of starting node
                if explored[curnode] is None:
                    continue

                # Skip bad paths that were enqueued before finding a better one
                qcost, h = enqueued[curnode]
                if qcost < dist:
                    continue

            explored[curnode] = parent
            
            for neighbor, w in self.get_neighbours_for_astar(curnode).items():
                ncost = dist + w
                if neighbor in enqueued:
                    qcost, h = enqueued[neighbor]

                    #self.print("qcost, h: " + str((qcost, h)))
                    # if qcost <= ncost, a less costly path from the
                    # neighbor to the source was already determined.
                    # Therefore, we won't attempt to push this neighbor
                    # to the queue
                    if qcost <= ncost:
                        continue
                else:
                    h = distance(neighbor, target)

                enqueued[neighbor] = ncost, h
                push(queue, (ncost + h, next(c), neighbor, ncost, curnode))
            
        self.print("No path found using Astar. Using RRT.")
        self.print((start_point, end_point))
        return self.RRT(start_point, end_point)

    def AstarSPT(self, path):
        ''' Removes waypoints from a path to make it smoother.
        Args:
            path: Nx3 array with waypoints
        Returns:
            The smooth path as a Nx3 NumPy array with points 
        '''
        k = 0
        n = len(path)
        start_point = path[0]
        smooth_path = np.array([start_point])
        point_k = start_point
        goal = path[-1]

        while not np.array_equal(point_k, goal):
            for i in reversed(range(k+1, n)):
                from_point = point_k
                to_point = path[i]
                if self.is_valid_step(from_point, to_point):
                    smooth_path = np.append(smooth_path, [to_point], axis=0)
                    point_k = to_point
                    k = i
                    break
                    
            if k != i:
                smooth_path = np.append(smooth_path, [path[k+1]], axis=0)
                k += 1
                point_k = path[k]   
            
        return smooth_path

    def get_neighbours_for_astar(self, curr_node):
        ''' Help function for Astar to find neighbours to a given node 
        Args:
            curr_node: Node as index in self.astar_points
        Returns:
            Valid neighbours to curr_node as a dictionary, with index in
            self.astar_points as key and distance to given node as value. 
        '''
        current_point = self.astar_points[curr_node]
        neighbours = {}
        nbr_of_neighbours = 8

        for direction in range(nbr_of_neighbours):
            angle = direction/nbr_of_neighbours*np.pi*2
            x = current_point[0] + np.cos(angle) * ASTAR_STEP_SIZE
            y = current_point[1] + np.sin(angle) * ASTAR_STEP_SIZE
            z = current_point[2]
            pos = np.array([x, y, z])
            
            nearest_point = self.traversable_pcd.find_k_nearest(pos, 1)[0]

            if self.is_valid_step(current_point, nearest_point):
                
                node = self.get_node(nearest_point)

                if node is False:
                    node = len(self.astar_points)
                    self.astar_points = np.append(self.astar_points, [nearest_point], axis=0)

                if node == curr_node:
                    continue

                neighbours[node] = np.linalg.norm(current_point - nearest_point)

        return neighbours


    def get_node(self, point):
        ''' Finds the node (index in self.astar_points) that is close to a given point.
        Args:
            point: A [x,y,z] array with the position of the point
        Returns:
            Index of the closest point in self.astar_points or False if no point found nearby.
        '''
        distance_to_existing_nodes = np.linalg.norm(self.astar_points - point, axis=1)
        closest_node = np.argmin(distance_to_existing_nodes)
        distance = distance_to_existing_nodes[closest_node]
        if distance < 0.8 * ASTAR_STEP_SIZE:
            return closest_node

        return False
        
    def new_point_towards(self, start_point, end_point, step_size):
        ''' Goes a step in a direction and returns the closest point from that position,
        Args:
            start_point: Starting point as [x,y,z] array
            end_point: Point defining the direction as [x,y,z] array
            step_size: Length of the step to make towards end_point.
        Returns:
            Closest traversable point as [x,y,z] from the end position of the step.         
        '''
        if np.linalg.norm(end_point - start_point) < step_size:
            return end_point

        direction = self.get_direction_vector(start_point, end_point)
        new_pos = start_point + step_size * direction
        return self.traversable_pcd.find_k_nearest(new_pos, 1)[0]   

    def new_point_at_angle(self, start_point, angle, step_size):
        ''' Goes a step in a direction and returns the closest point from that position,
        Args:
            start_point: Starting point as [x,y,z] array
            end_point: Point defining the direction as [x,y,z] array
            step_size: Length of the step to make towards end_point.
        Returns:
            Closest traversable point as [x,y,z] from the end position of the step.         
        '''
        x = start_point[0] + np.cos(angle) * step_size
        y = start_point[1] + np.sin(angle) * step_size
        z = start_point[2]
        pos = np.array([x, y, z])

        direction = self.get_direction_vector(start_point, pos)
        new_pos = start_point + step_size * direction
        return self.traversable_pcd.find_k_nearest(new_pos, 1)[0]   

    def get_direction_vector(self, start, goal):
        ''' Returns the direction vector between two points.
        Args:
            start: origin point as [x,y,z] array
            goal: end point as [x,y,z] array
        Returns:
            Vector of length 1.
        '''
        line_of_sight = goal - start
        return line_of_sight / np.linalg.norm(line_of_sight)
    

    def is_valid_step(self, from_point, to_point):
        ''' Checks if a step between two points is possible.
        Args:
            from_point: start point as [x,y,z] array
            to_point: end point as [x,y,z] array
        Returns:
            True if the step is valid, False otherwise.
        '''
        total_step_size = np.linalg.norm(to_point - from_point)
        if total_step_size == 0:
            return False

        if total_step_size <= STEP_SIZE:
            return True
        
        nbr_of_steps = int(np.ceil(total_step_size / STEP_SIZE))
        direction = self.get_direction_vector(from_point, to_point) * STEP_SIZE

        for step in range(1, nbr_of_steps):
            
            end_pos =  from_point + step * direction
            if self.traversable_pcd.distance_to_nearest(end_pos) > UNTRAVERSABLE_THRESHHOLD:
                return False
        return True