import numpy as np
import heapq
from typing import Union

class Graph:

    def __init__(self, adjacency_mat: Union[np.ndarray, str]):
        """
    
        Unlike the BFS assignment, this Graph class takes an adjacency matrix as input. `adjacency_mat` 
        can either be a 2D numpy array of floats or a path to a CSV file containing a 2D numpy array of floats.

        In this project, we will assume `adjacency_mat` corresponds to the adjacency matrix of an undirected graph.
    
        """
        if type(adjacency_mat) == str:
            self.adj_mat = self._load_adjacency_matrix_from_csv(adjacency_mat)
        elif type(adjacency_mat) == np.ndarray:
            self.adj_mat = adjacency_mat
        else: 
            raise TypeError('Input must be a valid path or an adjacency matrix')
        self.mst = None

    def _load_adjacency_matrix_from_csv(self, path: str) -> np.ndarray:
        with open(path) as f:
            return np.loadtxt(f, delimiter=',')

    def construct_mst(self):
        """
    
        TODO: Given `self.adj_mat`, the adjacency matrix of a connected undirected graph, implement Prim's 
        algorithm to construct an adjacency matrix encoding the minimum spanning tree of `self.adj_mat`. 
            
        `self.adj_mat` is a 2D numpy array of floats. Note that because we assume our input graph is
        undirected, `self.adj_mat` is symmetric. Row i and column j represents the edge weight between
        vertex i and vertex j. An edge weight of zero indicates that no edge exists. 
        
        This function does not return anything. Instead, store the adjacency matrix representation
        of the minimum spanning tree of `self.adj_mat` in `self.mst`. We highly encourage the
        use of priority queues in your implementation. Refer to the heapq module, particularly the 
        `heapify`, `heappop`, and `heappush` functions.

        """
        self.mst = None

        #Store input adjacency matrix as graph
        graph = self.adj_mat
        
        #Check if graph is connected
        if np.sum(graph) == 0:
            raise ValueError("There are no edges. Graph is not connected")
        
        #Initialize mst with zeros
        mst = np.zeros_like(graph)
        #Get total number of nodes to iterate through
        num_nodes = len(graph)
        #Initialize random start node
        start = np.random.randint(0, num_nodes)
        #Initialize set of visited nodes with start node
        visited = set([start])
        #Initialize heap to store edges
        edge_heap = []

        #Fill edge_heap with edges and weights from start node
        for n in range(num_nodes):
            weight = graph[start,n]
            if weight == 0: #if weight is 0, not connected and continue
                continue
            heapq.heappush(edge_heap, (weight, start, n)) #else, add weight, start node, neighbor node to edge_heap
            heapq.heapify(edge_heap) #order heap with heapify

        #visit neighbor node to start node
        while len(visited) != num_nodes:
            #check if heap is empty
            #if len(edge_heap) == 0:
                #raise ValueError('Heap is empty. Nodes are not connected')
           
            #visit neighbor node(to) with lowest weight edge from ordered edge_heap
            weight, start, to = heapq.heappop(edge_heap)
            heapq.heapify(edge_heap) #order heap
            if to in visited: #check if neighbor node has been visited, continue to end
                continue
            visited.add(to) #add node to visited set
            mst[start,to] = mst [to, start] = weight #update mst with corresponding edge nodes and weight

            #do this all again by iterating over rest of the nodes
            for n in range(num_nodes):
                weight = graph[to,n] #new start node/current node from last neighbor node. find weight
                if weight != 0:
                    heapq.heappush(edge_heap, (weight, to, n)) #store weight, curr node, end node
                    heapq.heapify(edge_heap) #order heap
        
        #return completed mst
        self.mst = mst





            
        

        


