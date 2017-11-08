import random
import numpy as np

ACTIONS = ['u', 'd', 'l', 'r']
ACTIONS_2 = ['u', 'd', 'l', 'r', 'ne', 'nw', 'sw', 'se']

_X = 1
_Y = 0


def neighbors(f, s, actions):
    """
    Returns the list of neighbors to the current state.

    :param f: The transition function.
    :param s: The current state.
    :param actions: The list of actions.
    """

    n = []
    for a in actions:
        s_prime = f(s, a)

        if s_prime != s:
            n.append(s_prime)

    return n


def action(s, s_prime):
    """
    Returns the action needed to get from state 's' to state 's_prime'.

    :param s: The current state.
    :param s_prime: The state being transitioned to.
    """

    np_s = np.array(s)
    np_sp = np.array(s_prime)

    v = np_sp - np_s
    if np.array_equal(v, [-1, 0]):
        return 'u'
    elif np.array_equal(v, [1, 0]):
        return 'd'
    elif np.array_equal(v, [0, -1]):
        return 'l'
    elif np.array_equal(v, [0, 1]):
        return 'r'
    elif np.array_equal(v, [-1, 1]):
        return 'ne'
    elif np.array_equal(v, [-1, -1]):
        return 'nw'
    elif np.array_equal(v, [1, 1]):
        return 'se'
    elif np.array_equal(v, [1, -1]):
        return 'sw'


class MDP(object):
    """
    A base class representing a Markov Decision Problem.
    """

    def __init__(self, gridmap, prob_list, actions=ACTIONS):

        self.gridmap = gridmap
        self.prob_commanded = prob_list[0]
        self.prob_neighboring = prob_list[1]
        self.prob_orthogonal = prob_list[2]
        self.actions = actions

    def _transition(self, s, a):

        # Ensure action stays on the board
        new_pos = list(s[:])
        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.gridmap.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.gridmap.cols - 1:
                new_pos[_X] += 1
        elif a == 'ne':
            if s[_Y] > 0 and s[_X] < self.gridmap.cols - 1:
                new_pos[_Y] -= 1
                new_pos[_X] += 1
        elif a == 'nw':
            if s[_Y] > 0 and s[_X] > 0:
                new_pos[_Y] -= 1
                new_pos[_X] -= 1
        elif a == 'se':
            if s[_Y] < self.gridmap.rows - 1 and s[_X] < self.gridmap.cols - 1:
                new_pos[_Y] += 1
                new_pos[_X] += 1
        elif a == 'sw':
            if s[_Y] < self.gridmap.rows - 1 and s[_X] > 0:
                new_pos[_Y] += 1
                new_pos[_X] -= 1
        else:
            print 'Unknown action:', str(a)

            # Test if new position is clear
        if self.gridmap.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)

        return s_prime

    def is_corner(self, index):

        if (index[0] == 0 or index[0] == self.gridmap.rows - 1) and (
                        index[1] == 0 or index[1] == self.gridmap.cols - 1):
            return True
        else:
            return False

    def _is_orthogonal_action(self, a):

        if a == 'u' or a == 'd' or a == 'l' or a == 'r':
            return True
        else:
            return False

    def _is_neighboring_action(self, a):

        if a == 'ne' or a == 'nw' or a == 'se' or a == 'sw':
            return True
        else:
            return False

    def _transition_orthogonally(self, a):

        if a == 'u' or a == 'd':
            return ['l', 'r']
        elif a == 'l' or a == 'r':
            return ['u', 'd']
        elif a == 'nw' or a == 'se':
            return ['ne', 'sw']
        elif a == 'ne' or a == 'sw':
            return ['nw', 'se']
        else:
            print 'Unknown action:', str(a)

    def _transition_neighboring(self, a):

        if a == 'u':
            return ['ne', 'nw']
        elif a == 'd':
            return ['se', 'sw']
        elif a == 'l':
            return ['nw', 'sw']
        elif a == 'r':
            return ['ne', 'se']
        elif a == 'nw':
            return ['u', 'l']
        elif a == 'ne':
            return ['u', 'r']
        elif a == 'sw':
            return ['l', 'd']
        elif a == 'se':
            return ['d', 'r']
        else:
            print 'Unknown action:', str(a)

    def _random_action(self, command):
        """
        Generates a random action according to the transition probabilities of this MDP.

        :param command: The commanded action.
        :return: An action (either neighboring or orthogonal) chosen randomly.
        """

        pc = 1 - random.random()
        r = 1 - random.random()

        if pc <= self.prob_commanded:
            return command

        elif pc > self.prob_commanded and pc <= self.prob_commanded + (2 * self.prob_neighboring):
            neighboring_actions = self._transition_neighboring(command)

            if r > 0.5:
                return neighboring_actions[0]
            else:
                return neighboring_actions[1]
        else:
            orthogonal_actions = self._transition_orthogonally(command)

            if r > 0.5:
                return orthogonal_actions[0]
            else:
                return orthogonal_actions[1]

    def transition(self, s, a, simulate=False):
        """

        :param s: A tuple describing the state as (row, col) position on the grid.
        :param a: The action to be performed from state s.
        :param simulate: A boolean flag indicating if this function should simulate the transition or not.

        :return: A probability distribution over subsequent states.
        """

        if simulate:

            # The goal is an absorbing state
            if self.gridmap.is_goal(s):
                return s
            else:
                return self._transition(s, self._random_action(a))
        else:

            if self.gridmap.is_goal(s):
                return [(s, 1)]

            distribution = [(self._transition(s, a), self.prob_commanded)]

            orthogonal_actions = self._transition_orthogonally(a)
            for act in orthogonal_actions:
                distribution.append((self._transition(s, act), self.prob_orthogonal))

            if self.actions == ACTIONS_2:
                neighboring_actions = self._transition_neighboring(a)
                for act in neighboring_actions:
                    distribution.append((self._transition(s, act), self.prob_neighboring))

            return distribution

    def run(self, plan):
        """
        Runs the MDP simulation on the BFS result from the deterministic system.

        :return: A path modeled with the uncertainty of the simulator of this MDP.
        """

        new_plan = []
        s_prime = self.gridmap.init_pos

        for i in range(0, len(plan) - 1):
            action = plan[i + 1]
            s_prime = self.transition(s_prime, action, simulate=True)

            new_plan.append(s_prime)

        return new_plan


class MDPValueIterator(MDP):
    """
    A discounted MDP solved using Value Iteration.
    """

    def __init__(self, gridmap, prob_list, rewards, discount, epsilon=0.001,
                 max_iter=1000, actions=ACTIONS, absorb=True):

        MDP.__init__(self, gridmap, prob_list, actions)

        self.r_goal = rewards[0]
        self.r_corners = rewards[1]
        self.r_others = rewards[2]
        self.discount = discount
        self.epsilon = epsilon
        self.absorb = absorb

        self.V = np.zeros((self.gridmap.rows, self.gridmap.cols))  # The value matrix
        self.R = np.zeros((self.gridmap.rows, self.gridmap.cols))  # The reward matrix

        # Initialize the states
        self.states = [(r, c) for r in range(self.gridmap.rows) for c in range(self.gridmap.cols) if
                       not self.gridmap.occupancy_grid[r][c]]

        # Initialize the reward matrix
        for r in range(self.gridmap.rows):
            for c in range(self.gridmap.cols):
                if self.gridmap.is_goal((r, c)):
                    self.R[r][c] = self.r_goal
                elif self.is_corner((r, c)) and self.r_corners != 0:
                    self.R[r][c] = self.r_corners
                else:
                    self.R[r][c] = self.r_others

        self.max_iter = max_iter

        self.iter = 0
        self.policy = {}

    def run(self):
        """
        Run the Value Iteration algorithm
        """

        while True:

            v_prev = self.V.copy()

            for s in self.states:

                v_max = float("-inf")
                if self.gridmap.is_goal(s) and self.absorb:
                    v_max = self.R[s]
                else:
                    for a in self.actions:
                        P = self.transition(s, a)

                        v_s = 0
                        for e in P:
                            s_prime = e[0]
                            p_prime = e[1]
                            v_prime = v_prev[s_prime]

                            v_s += p_prime * (self.R[s] + self.discount * v_prime)

                        if v_s > v_max:
                            v_max = v_s
                            self.policy[s] = a

                self.V[s] = v_max

            variation = np.absolute(self.V - v_prev).max()

            if variation < self.epsilon:
                print("Iterating stopped. Epsilon-optimal policy found in ({0}) iterations.".format(self.iter))
                break
            elif self.iter == self.max_iter:
                print("Iterating stopped. Maximum iterations exceeded.")
                break

            self.iter += 1

        return self.V, self.policy


class MDPPolicyIterator(MDP):
    """
    A discounted MDP solved using Policy Iteration.
    """

    def __init__(self, gridmap, prob_list, rewards, discount, policy=None, epsilon=0.001, max_iter=1000, actions=ACTIONS,
                 absorb=True, random_update=False):
        MDP.__init__(self, gridmap, prob_list, actions=actions)

        self.r_goal = rewards[0]
        self.r_corners = rewards[1]
        self.r_others = rewards[2]
        self.discount = discount
        self.epsilon = epsilon
        self.absorb = absorb
        self.random_update = random_update

        self.policy = {}

        self.V = np.zeros((self.gridmap.rows, self.gridmap.cols))  # The value matrix
        self.R = np.zeros((self.gridmap.rows, self.gridmap.cols))  # The reward matrix

        # Initialize the states
        self.states = [(r, c) for r in range(self.gridmap.rows) for c in range(self.gridmap.cols) if
                       not self.gridmap.occupancy_grid[r][c]]

        # Initialize the reward matrix and policy
        for r in range(self.gridmap.rows):
            for c in range(self.gridmap.cols):
                if self.gridmap.is_goal((r, c)):
                    self.R[r][c] = self.r_goal
                elif self.is_corner((r, c)) and self.r_corners != 0:
                    self.R[r][c] = self.r_corners
                else:
                    self.R[r][c] = self.r_others

                # If the policy is None, generate a random policy
                if policy is None:
                    self.policy[(r, c)] = random.choice(self.actions)
                else:
                    self.policy[(r, c)] = policy

        self.max_iter = max_iter
        self.iter = 0

    def update_policy(self, v_policy):

        policy_prime = {}
        for s in self.states:

            if self.gridmap.is_goal(s):
                policy_prime[s] = None
            else:

                if self.random_update:
                    act = list(self.actions)
                    random.shuffle(act)
                else:
                    act = self.actions

                v_max = float("-inf")
                for a in act:
                    P = self.transition(s, a)

                    v_s = 0
                    for e in P:
                        s_prime = e[0]
                        p_prime = e[1]
                        v_prime = v_policy[s_prime]

                        v_s += p_prime * (self.R[s] + self.discount * v_prime)

                    if v_s > v_max:
                        v_max = v_s
                        policy_prime[s] = a

        return policy_prime

    def run(self):
        """
        Run the Policy Iteration algorithm
        """

        v_policy = self.V.copy()
        while True:

            i = 0
            done = False
            while not done:
                i += 1

                v_prev = v_policy.copy()
                for s in self.states:

                    if self.gridmap.is_goal(s) and self.absorb:
                        v_s = self.R[s]
                    else:
                        P = self.transition(s, self.policy[s])

                        v_s = 0
                        for e in P:
                            s_prime = e[0]
                            p_prime = e[1]
                            v_prime = v_prev[s_prime]

                            v_s += p_prime * (self.R[s] + self.discount * v_prime)

                    v_policy[s] = v_s

                variation = np.absolute(v_policy - v_prev).max()

                if variation < self.epsilon:
                    done = True
                    print("Iterating stopped. Epsilon-optimal policy found in ({0}) iterations.".format(i))
                elif i == self.max_iter:
                    done = True
                    print("Iterating stopped. Maximum iterations exceeded.")

            policy_prime = self.update_policy(v_policy)

            if self.policy == policy_prime:
                print("Iterating stopped. Unchanging policy found in ({0}) iterations.".format(self.iter))
                break
            elif self.iter == self.max_iter:
                print("Iterating stopped. An unchanging policy was not found within the maximum number of iterations.")
                break

            self.policy = policy_prime

            self.iter += 1

        return v_policy, self.policy
