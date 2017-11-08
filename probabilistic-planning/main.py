import argparse
import gridmap
import mdp

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', "--input", required=True, help="The environment map file path.")
    parser.add_argument('-d', "--diagonals", action="store_true", default=False,
                        help="Use diagonal actions in addition to the four cardinal directions.")
    parser.add_argument('-p', "--probs", required=True,
                        help="A comma delimited list of probabilities (command,neighboring,orthogonal).")

    parser.add_argument('-a', '--algorithm', required=True, default='vi', choices=['mdp', 'vi', 'pi'],
                        help='The algorithm which should be run.')

    # VI and PI specific command line arguments
    parser.add_argument("--absorb", action='store_true', default=True,
                        help="Sets the goal as an absorbing state, so no transitions are computed over it.")
    parser.add_argument("--rewards", help="A comma delimited list of rewards (applies to VI and PI only).")
    parser.add_argument("--discount", type=float, help="The discount factor to use (applies to VI and PI only).")
    parser.add_argument("--epsilon", type=float, default=0.01, help="The epsilon to converge to.")

    # PI specific command line arguments
    parser.add_argument("--policy", default=None, help="The policy to use for each state.")
    parser.add_argument("--random_update", action='store_true', default=False, help="Break ties by randomizing the action.")

    args = parser.parse_args()
    prob_list = [float(item) for item in args.probs.split(',')]

    if args.rewards:
        rewards = [int(item) for item in args.rewards.split(',')]

    if args.diagonals:
        a = mdp.ACTIONS_2
    else:
        a = mdp.ACTIONS

    # Create the grid using the supplied input file
    grid = gridmap.GridMap(map_path=args.input)

    # Run the desired algorithm
    if args.algorithm == 'mdp':
        plan, visited = gridmap.bfs(grid.init_pos, grid.transition, grid.is_goal, a)

        mp = mdp.MDP(grid, prob_list, actions=a)
        p = mp.run(plan[1])
        grid.display_map(p)

    elif args.algorithm == 'vi':
        vi = mdp.MDPValueIterator(grid, prob_list, rewards, args.discount, epsilon=args.epsilon, actions=a,
                                  absorb=args.absorb)
        V, policy = vi.run()
        grid.display_vi_results(V, policy)

    else:
        pi = mdp.MDPPolicyIterator(grid, prob_list, rewards, args.discount, policy=args.policy, epsilon=args.epsilon,
                                   actions=a, absorb=args.absorb, random_update=args.random_update)
        V, policy = pi.run()
        grid.display_vi_results(V, policy)
