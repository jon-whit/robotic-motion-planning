#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)


class RRTSearchTree:
    def __init__(self, init):
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def add_node(self, node, parent):
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def combine_trees(self, Ta, Tb):

        for node in Ta.nodes:
            if node.parent is not None:
                self.add_node(node, parent=node.parent)
            else:
                self.nodes.append(Ta.root)
                for child in node.children:
                    self.edges.append((node.state, child.state))

        for node in Tb.nodes:
            if node.parent is not None:
                self.add_node(node, parent=node.parent)
            else:
                self.nodes.append(Tb.root)
                for child in node.children:
                    self.edges.append((node.state, child.state))

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent

        path.append(n.state)
        path.reverse()
        return path

class RRT:

    def __init__(self, num_samples, num_dimensions=2, step_length = 1, lims = None,
                 connect_prob = 0.05, collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)

        for k in range(0, self.K):
            s_prime = self.sample(self.goal)
            n_prime = self.extend(s_prime)

            if n_prime is not None and tuple(n_prime.state) == tuple(self.goal):
                return self.T.get_back_path(n_prime)

        return None

    def build_rrt_connect(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)

        for k in range(0, self.K):

            s_prime = self.sample(self.goal)

            n_prime = self.extend(s_prime)

            # while not in collision or while sample is not met or until goal is met
            #
            while n_prime is not None and tuple(n_prime.state) != tuple(s_prime) and tuple(n_prime.state) != tuple(self.goal):
                n_prime = self.extend(s_prime)

            if n_prime is not None and tuple(n_prime.state) == tuple(self.goal):
                return self.T.get_back_path(n_prime)

        return None

    def sample(self, bias_target):
        '''
        Sample a new configuration and return
        '''
        # Return goal with connect_prob probability

        if 1 - np.random.uniform() <= self.connect_prob:
            return bias_target
        else:

            if self.n == 2:
                randx = np.random.uniform(self.limits[0][0], self.limits[0][1] + 1)
                randy = np.random.uniform(self.limits[1][0], self.limits[1][1] + 1)
                pt = np.array([randx, randy])
            elif self.n == 3:
                theta1 = np.random.uniform(self.limits[0][0], self.limits[0][1])
                theta2 = np.random.uniform(self.limits[1][0], self.limits[1][1])
                theta3 = np.random.uniform(self.limits[2][0], self.limits[2][1])
                pt = np.array([theta1, theta2, theta3])

            return pt

    def extend(self, q):
        '''
        Perform rrt extend operation.
        q - new configuration to extend towards
        '''

        (nearest_neighbor, distance) = self.T.find_nearest(self.goal)

        new_node = None
        if distance <= self.epsilon:
            # Add the goal to the tree with its parent
            new_node = TreeNode(self.goal, parent=nearest_neighbor)
            self.T.add_node(new_node, parent=nearest_neighbor)
        else:
            # Find the nearest point on the tree to the new sample
            (nearest_neighbor, distance) = self.T.find_nearest(q)

            if distance <= self.epsilon:
                s_prime = q
            else:
                # Move towards the sample epsilon amount
                v = (q - nearest_neighbor.state)/np.linalg.norm(q - nearest_neighbor.state)
                s_prime = nearest_neighbor.state + (v*self.epsilon)

            # Keep the new state if it is not in collision
            if not self.in_collision(s_prime):
                new_node = TreeNode(s_prime, parent=nearest_neighbor)
                self.T.add_node(new_node, parent=nearest_neighbor)

        return new_node

    def build_bidirectional_rrt(self, init, goal):

        self.init = np.array(init)
        self.goal = np.array(goal)

        Ta = RRTSearchTree(init)
        Tb = RRTSearchTree(goal)


        self.T = RRTSearchTree(init)

        for k in range(0, self.K):
            qrand = self.sample_bi()

            ret, qtarget = self.extend_bi(Ta, qrand)
            if not ret == _TRAPPED:
                while ret != _TRAPPED:
                    ret, qnew = self.extend_bi(Tb, qtarget.state)

                    if ret == _REACHED:
                        self.T.combine_trees(Ta, Tb)
                        return self.path(Ta, qtarget, Tb, qnew)

            temp = Ta
            Ta = Tb
            Tb = temp

        self.T.combine_trees(Ta, Tb)
        return None

    def path(self, Ta, qtarget, Tb, qnew):
        """
        Returns the path between to connected trees with qnew being the common node between them.

        :param Ta:
        :param Tb:
        :param qnew:
        :return:
        """

        if tuple(Ta.root.state) == tuple(self.init):
            Tb_path = Tb.get_back_path(qnew)
            Tb_path.reverse()

            return Ta.get_back_path(qtarget) + Tb_path
        else:
            Ta_path = Ta.get_back_path(qtarget)
            Ta_path.reverse()

            return Tb.get_back_path(qnew) + Ta_path

    def sample_bi(self):

        if self.n == 2:
            randx = np.random.uniform(self.limits[0][0], self.limits[0][1] + 1)
            randy = np.random.uniform(self.limits[1][0], self.limits[1][1] + 1)
            pt = np.array([randx, randy])
        elif self.n == 3:
            theta1 = np.random.uniform(self.limits[0][0], self.limits[0][1])
            theta2 = np.random.uniform(self.limits[1][0], self.limits[1][1])
            theta3 = np.random.uniform(self.limits[2][0], self.limits[2][1])
            pt = np.array([theta1, theta2, theta3])

        return pt

    def extend_bi(self, T, qrand):

        tnew = None

        # Extend the tree to the sampled configuration point (qrand)
        (nearest_neighbor, distance) = T.find_nearest(qrand)

        if distance <= self.epsilon:
            tnew = TreeNode(qrand, parent=nearest_neighbor)
            res = _REACHED
        else:
            # Move towards the sample epsilon amount
            v = (qrand - nearest_neighbor.state)/np.linalg.norm(qrand - nearest_neighbor.state)
            qnew = nearest_neighbor.state + (v*self.epsilon)

            # Keep the new state if it is not in collision
            if not self.in_collision(qnew):
                tnew = TreeNode(qnew, parent=nearest_neighbor)
                T.add_node(tnew, parent=nearest_neighbor)
                res = _ADVANCED
            else:
                res = _TRAPPED

        return res, tnew

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect=False):
    '''
    create an instance of PolygonEnvironment from a description file and plan a path from start to goal on it using an RRT

    num_samples - number of samples to generate in RRT
    step_length - step size for growing in rrt (epsilon)
    env - path to the environment file to read
    connect - If True run rrt_connect

    returns plan, planner - plan is the set of configurations from start to goal, planner is the rrt used for building the plan
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)

    dims = len(pe.start)
    start_time = time.time()

    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.05,
              collision_func=pe.test_collisions)
    if connect:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    else:
        plan = rrt.build_rrt(pe.start, pe.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    return plan, rrt
