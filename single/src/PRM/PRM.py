
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path
        self.robot_radius = 5

    def check_collision(self, p1, p2):
        obstacle_collision = False
        path = np.linspace([p1[0], p1[1]], [p2[0], p2[1]], num=10, dtype=int)
        for pt in path:
            for i in range(pt[0]-self.robot_radius, pt[0]+self.robot_radius):
                for j in range(pt[1]-self.robot_radius, pt[1]+self.robot_radius):
                    if self.map_array[i,j] == 0:
                        obstacle_collision = True
                        break
        return obstacle_collision

    def dis(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        self.graph.clear()

        sample_list = [(12.5,12.5),(12.5,37.5),(12.5,62.5),(12.5,87.5),(12.5,112.5),(12.5,137.5),(12.5,162.5),(12.5,187.5),(12.5,212.5),\
            (37.5,12.5),(37.5,112.5),(37.5,212.5),(62.5,12.5),(62.5,112.5),(62.5,212.5),\
            (87.5,12.5),(87.5,37.5),(87.5,62.5),(87.5,87.5),(87.5,112.5),(87.5,137.5),(87.5,162.5),(87.5,187.5),(87.5,212.5),\
            (112.5,12.5),(112.5,112.5),(112.5,212.5),(137.5,12.5),(137.5,112.5),(137.5,212.5),\
            (162.5,12.5),(162.5,37.5),(162.5,62.5),(162.5,87.5),(162.5,112.5),(162.5,137.5),(162.5,162.5),(162.5,187.5),(162.5,212.5)]
        for sample in sample_list:
            self.samples.append(sample)

        pairs = []
        p_ids = []
        KDtree = spatial.KDTree(np.array(self.samples))
        KDtree_pairs = KDtree.query_pairs(30)
        for pair in KDtree_pairs:
            point1 = self.samples[pair[0]]
            point2 = self.samples[pair[1]]
            distance = self.dis(point1, point2)
            if self.check_collision(point1, point2):
                continue
            if pair[0] not in p_ids:
                p_ids.append(pair[0])
            if pair[1] not in p_ids:
                p_ids.append(pair[1])
            pairs.append((pair[0], pair[1], distance))

        self.graph.add_nodes_from(p_ids)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))

    def search(self, start, goal, sampling_method="uniform"):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        start_pairs = []
        goal_pairs = []

        start_goal_radius = 30

        KDtree = spatial.KDTree(np.array(self.samples))
        KDtree_start_pairs = spatial.KDTree.query_ball_point(KDtree, start, start_goal_radius)
        KDtree_goal_pairs = spatial.KDTree.query_ball_point(KDtree, goal, start_goal_radius)
        for s in KDtree_start_pairs:
            if self.check_collision(start, self.samples[s]):
                continue
            distance = self.dis(start, self.samples[s])
            start_pairs.append(('start', s, distance))
        for g in KDtree_goal_pairs:
            if self.check_collision(goal, self.samples[g]):
                continue
            distance = self.dis(goal, self.samples[g])
            start_pairs.append(('goal', g, distance))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            print("PATH", self.path)
            gazebo_path = []
            for i in self.path:
                if i=='start':
                    pixel_coord = start
                elif i=='goal':
                    pixel_coord = goal
                else:
                    pixel_coord = self.samples[i]
                x = pixel_coord[1]/25 - 4.5
                y = (200-pixel_coord[0])/25 - 4.5
                gazebo_coord = (x,y)
                gazebo_path.append(gazebo_coord)
            print(gazebo_path)
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
