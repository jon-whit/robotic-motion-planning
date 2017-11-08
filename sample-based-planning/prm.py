import numpy as np
import copy
from scipy import spatial


class GraphNode:
    """
    Represents a node in a graph structure.

    Each node has a state and parent associated with it.
    """
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent


class PRMGraph:
    """
    A graph class for a Probabilistic Roadmap.

    A PRMGraph maintains an adjacency matrix and a list of edges.
    """

    def __init__(self):
        self.adjmtx = {}
        self.edges = []

    def add_node(self, state, neighbors):
        """
        Adds a node and its neighbors to the adjacency list backing the graph.

        :param state: The new state being added to the graph.
        :param neighbors: The neighboring states of the new state being added.
        """

        if tuple(state) not in self.adjmtx.keys():
            self.adjmtx[tuple(state)] = []

        # Use a tuple instead of a numpy array because of hashing mutability rules
        for neighbor in neighbors:
            self.adjmtx[tuple(state)].append(neighbor)
            self.edges.append((state, neighbor))

    def get_states_and_edges(self):
        """
        Gets the current states and edges in the graph.

        :return: A 2-tuple with the first element being graph states and the second being the edges.
        """

        states = np.array(self.adjmtx.keys())
        return states, self.edges

    def get_path(self, start, goal):
        """
        Gets a path between the start and goal position within the graph (if one exists) using a BFS graph search.

        :param start: The starting state.
        :param goal: The goal state.
        :return: An ordered list of states that were traversed to get to the goal or None if no path existed.
        """

        frontier = [GraphNode(start)]
        visited = set()

        while frontier:
            n_i = frontier.pop(0)

            if tuple(n_i.state) not in visited:
                visited.add(tuple(n_i.state))

            if tuple(n_i.state) == tuple(goal):
                return self.get_back_path(n_i)

            for neighbor in self.adjmtx[tuple(n_i.state)]:

                if tuple(neighbor) not in visited:
                    frontier.append(GraphNode(neighbor, n_i))

        return None

    def get_back_path(self, n):

        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent

        path.append(n.state)
        path.reverse()

        return path


class PRM:

    def __init__(self, num_samples, num_neighbors, num_dimensions=2, step_length=1, lims=None, gaussian=False,
                 variance=1, collision_func=None):
        """
        Initialize a Probabalistic Roadmap (PRM) planning instance.
        """
        self.K = num_samples
        self.num_neighbors = num_neighbors
        self.n = num_dimensions
        self.epsilon = step_length
        self.limits = lims
        self.gaussian = gaussian
        self.variance = variance

        self.presampled = False
        self.samples = []

        if collision_func is None:
            self.in_collision = self.fake_in_collision
        else:
            self.in_collision = collision_func

    def build_prm(self):
        """
        Build the PRM without attaching the start and goal states.
        """

        # Generate random points uniformly within the configuration space
        if self.n == 2:
            randx = np.random.uniform(self.limits[0][0], self.limits[0][1] + 1, self.K)
            randy = np.random.uniform(self.limits[1][0], self.limits[1][1] + 1, self.K)
            pts = np.column_stack((randx, randy))
        elif self.n == 3:
            theta1 = np.random.uniform(self.limits[0][0], self.limits[0][1] + 1, self.K)
            theta2 = np.random.uniform(self.limits[1][0], self.limits[1][1] + 1, self.K)
            theta3 = np.random.uniform(self.limits[2][0], self.limits[2][1] + 1, self.K)
            pts = np.column_stack((theta1, theta2, theta3))

        kept_samples = []

        if self.gaussian:

            # Use a Gaussian sampling strategy
            for pt in pts:

                if self.n == 2:
                    gaussian_randx = np.random.normal(pt[0], scale=self.variance)
                    gaussian_randy = np.random.normal(pt[1], scale=self.variance)

                    if gaussian_randx > self.limits[0][1]:
                        gaussian_randx = self.limits[0][1]

                    if gaussian_randx < self.limits[0][0]:
                        gaussian_randx = self.limits[0][0]

                    if gaussian_randy > self.limits[1][1]:
                        gaussian_randy = self.limits[1][1]

                    if gaussian_randy < self.limits[1][0]:
                        gaussian_randy = self.limits[1][0]

                    gaussian_pt = np.array([gaussian_randx, gaussian_randy])
                elif self.n == 3:
                    gaussian_t1 = np.random.normal(pt[0], scale=self.variance)
                    gaussian_t2 = np.random.normal(pt[1], scale=self.variance)
                    gaussian_t3 = np.random.normal(pt[2], scale=self.variance)
                    gaussian_pt = np.array([gaussian_t1, gaussian_t2, gaussian_t3])

                # If only one of the sampled points is collision free, then keep the
                # collision free sample.
                if self.in_collision(pt):
                    if not self.in_collision(gaussian_pt):
                        kept_samples.append(gaussian_pt)
                else:
                    if self.in_collision(gaussian_pt):
                        kept_samples.append(pt)

        else:
            # Keep only those samples that are collision free
            kept_samples = [pt for pt in pts if not self.in_collision(pt)]


        # Create the initial graph
        self.T = PRMGraph()

        # Use a KDTree to find the closest neighbors to the sampled points. Add the connected neighbors to the graph.
        i = 0
        for pt in kept_samples:
            foreign_neighbors = np.delete(kept_samples, i, 0)
            closest_neighbors = foreign_neighbors[spatial.KDTree(foreign_neighbors).query(pt, k=self.num_neighbors)[1]]

            connected_neighbors = self.filter_unconnectable(pt, closest_neighbors)
            self.T.add_node(pt, connected_neighbors)

            i += 1

        return copy.deepcopy(self.T)

    def find_plan(self, start, goal, roadmap):
        """
        Attaches the start and goal states to the existing PRM, and attempts to find a path between them.

        :param start: The starting state to attach to the PRM.
        :param goal: The goal state to attach to the PRM.
        :return: An ordered list of states that were traversed to get to the goal or None if no path existed.
        """

        self.goal = np.array(goal)
        self.start = np.array(start)

        # If the start node is not in the adjacency matrix, then add it
        if tuple(self.start) not in roadmap.adjmtx.keys():
            foreign_neighbors = np.array(roadmap.adjmtx.keys())

            # Find the closest neighbors to the start
            closest_neighbors = foreign_neighbors[spatial.KDTree(foreign_neighbors).query(self.start, k=self.num_neighbors)[1]]
            connected_neighbors = self.filter_unconnectable(self.start, closest_neighbors)
            roadmap.add_node(self.start, connected_neighbors)

        # If the goal node is not in the adjacency matrix, then add it
        if tuple(self.goal) not in roadmap.adjmtx.keys():
            foreign_neighbors = np.array(roadmap.adjmtx.keys())

            # Find the closest neighbors to the goal
            closest_neighbors = foreign_neighbors[spatial.KDTree(foreign_neighbors).query(self.goal, k=self.num_neighbors)[1]]
            connected_neighbors = self.filter_unconnectable(self.goal, closest_neighbors)
            roadmap.add_node(self.goal, connected_neighbors)

            for neighbor in connected_neighbors:
                roadmap.add_node(neighbor, [self.goal])

        # Search the map for a plan
        return roadmap.get_path(self.start, self.goal)

    def filter_unconnectable(self, pt, closest_neighbors):
        """
        Filters the unconnectable neighboring states from the specified point and its closest neighbors.

        :param pt: The source point.
        :param closest_neighbors: A list of destination points.
        :return: A list of neighboring states that are connectable.
        """
        return [neighbor for neighbor in closest_neighbors if self.line_search(pt, neighbor)]

    def line_search(self, pt, neighbor):
        """
        Performs an incremental line search collision test between the specified point and its neighbor.

        :param pt: The source point.
        :param neighbor: A destination point.
        :return: True if a collision free path between the source and destination points exists. False otherwise.
        """
        v = (neighbor - pt)/np.linalg.norm(neighbor - pt)
        l = np.linalg.norm(neighbor - pt)

        i = 0
        l_prime = 0
        while l_prime <= l:
            pt_prime = pt + (i*v*self.epsilon)

            if self.in_collision(pt_prime):
                return False

            l_prime = np.linalg.norm(pt_prime - pt)
            i += 1

        return True

    def fake_in_collision(self, q):
        """

        :param q:
        :return:
        """
        return False
