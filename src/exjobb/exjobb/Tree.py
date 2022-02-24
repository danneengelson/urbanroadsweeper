import networkx as nx
import numpy as np
from python_tsp.heuristics import solve_tsp_local_search

class Tree:
    ''' Class for handling calculation on a graph with nodes and edges
    '''

    def __init__(self, name = False):
        ''' 
        Args:
            name: Name of tree
        '''
        self.tree = nx.Graph()
        if name:
            self.tree = nx.Graph(name=name)
        self.nodes = np.array([])

    def add_node(self, point):
        ''' Adds a point to the tree by extending the self.nodes list and 
        adding a node to the tree
        Args:
            point: a [x,y,z] point
        Returns:
            The index of the new node
        '''
        if self.nodes.size == 0:
            self.nodes = np.array([point])
            self.tree.add_node(0)
            return 0
            
        idx = self.tree.number_of_nodes()
        self.tree.add_node(idx)
        self.nodes = np.append( self.nodes , [point], axis=0)
        return idx

    def nearest_node(self, pos):
        ''' Finds the nearest point in tree from a given position
        Args:
            pos: A [x,y,z] position
        Returns:
            The index of the node and [x,y,z] position of it's point.
        '''
        distances = np.linalg.norm(self.nodes - pos, axis=1)
        nearest_point_idx = np.argmin(distances)
        return nearest_point_idx, self.nodes[nearest_point_idx]

    def add_edge(self, node_from_idx, node_to_idx, weight):
        ''' Connect two nodes in the tree with a edge cost
        Args:
            node_from_idx: Index of the node where the edge starts
            node_to_idx: Index of the node where the edge ends
            weight: Cost of the edge
        '''
        self.tree.add_edge(node_from_idx, node_to_idx, weight=weight, tree=self.tree.name)

    def get_traveling_salesman_path(self, max_time=120):
        ''' Applies an approximal Traveling Salesman Algorithm that tries to finds 
        the cheapest path through every node in the tree.
        Args:
            max_time: The maximum computational time to search.
        Returns:
            List of nodes indices, in the order they should be visited
        '''
        max_idx = self.tree.number_of_nodes()
        weight_matrix = np.zeros( (max_idx, max_idx) )
        for node in range(max_idx):
            for other_node in range(max_idx):
                if node == other_node:
                    weight_matrix[node, other_node] = 0
                else:
                    weight_matrix[node, other_node] = self.tree[node][other_node]["weight"]
        
        weight_matrix[:, 0] = 0
        permutation, distance = solve_tsp_local_search(weight_matrix, max_processing_time=max_time)
        return permutation
