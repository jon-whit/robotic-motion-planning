import argparse
import graph_search as GS


def print_path(a, path):

    print "{0} Path Taken:".format(a)
    print "{0}\n".format(path)


def print_action(a, actions):
    print "{0} Actions Taken:".format(a)
    print "{0}\n".format(actions)


def print_visited(a, visited):
    print "{0} Visited:".format(a)
    print "{0}\n".format(visited)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', "--input", required=True, help="The input map file path.")
    parser.add_argument('-x', "--disp_path", action='store_true', help='Print and plot the results.')
    parser.add_argument('-d', '--diagonals', action='store_true', help='Use diagonal actions.')
    parser.add_argument('-a', '--algorithms', nargs='+', required=True, choices=['dfs', 'bfs', 'iddfs', 'ucs', 'astar'],
                        help='1 or more path finding algorithms to run.')
    parser.add_argument('--heuristic', nargs='?', choices=['euclidean', 'manhattan'], help='The heuristic which will be used for A*.')

    args = parser.parse_args()

    # Create the grid from the input file
    gridmap = GS.GridMap(args.input)

    # If diagonal actions are allowed, then pass them along
    if args.diagonals:
        actions = set(GS._ACTIONS_2)
    else:
        actions = set(GS._ACTIONS)

    # To show the results of the given option to screen.
    for a in args.algorithms:
        if a == 'dfs':
            backpathDFS, visitedDFS = GS.dfs(gridmap.init_pos, gridmap.transition, gridmap.is_goal, actions)

            if args.disp_path:
                print_path(a, backpathDFS[0])
                print_action(a, backpathDFS[1])
                print_visited(a, visitedDFS)
                gridmap.display_map(backpathDFS[0], visitedDFS)

        elif a == 'bfs':
            backpathBFS, visitedBFS = GS.bfs(gridmap.init_pos, gridmap.transition, gridmap.is_goal, actions)

            if args.disp_path:
                print_path(a, backpathBFS[0])
                print_action(a, backpathBFS[1])
                print_visited(a, visitedBFS)
                gridmap.display_map(backpathBFS[0], visitedBFS)

        elif a == 'iddfs':
            backpathIDDFS, visitedIDDFS = GS.iddfs(gridmap.init_pos, gridmap.transition, gridmap.is_goal, actions)

            if args.disp_path:
                print_path(a, backpathIDDFS[0])
                print_action(a, backpathIDDFS[1])
                print_visited(a, visitedIDDFS)
                gridmap.display_map(backpathIDDFS[0], visitedIDDFS)

        elif a == 'ucs':
            backpathUCS, visitedUCS = GS.uniform_cost_search(gridmap.init_pos, gridmap.transition, gridmap.is_goal, actions)

            if args.disp_path:
                print_path(a, backpathUCS[0])
                print_action(a, backpathUCS[1])
                print_visited(a, visitedUCS)
                gridmap.display_map(backpathUCS[0], visitedUCS)

        elif a == 'astar':

            if ('heuristic', 'euclidean') or ('heuristic', 'manhattan') in args._get_kwargs():
                heuristic = gridmap.euclidean_heuristic if args.heuristic == 'euclidean' else gridmap.manhattan_heuristic
                backpathAstar, visitedAstar = GS.a_star_search(gridmap.init_pos, gridmap.transition, gridmap.is_goal, actions, gridmap.euclidean_heuristic)

                if args.disp_path:
                    print_path(a, backpathAstar[0])
                    print_action(a, backpathAstar[1])
                    print_visited(a, visitedAstar)
                    gridmap.display_map(backpathAstar[0], visitedAstar)
            else:
                print "You must specify a heuristic to use for A*! For help, use 'main.py --help'."

