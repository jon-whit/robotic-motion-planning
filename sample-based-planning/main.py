import argparse
import copy
import matplotlib.pyplot as plotter
import collisions, rrt, prm
import ast
import os


def savefig(dir, filename):

    filepath = dir + filename

    if not os.path.exists(dir):
        os.makedirs(dir)

    i = 0
    while os.path.exists('{}-{:d}.png'.format(filepath, i)):
        i += 1

    plotter.savefig('{}-{:d}.png'.format(filepath, i))

if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    # Create the CLI using the ArgumentParser
    parser.add_argument("-e", "--environment", required=True, help="The txt file defining the environment.")
    parser.add_argument("-p", "--planner", required=True, choices=['prm', 'rrt'],
                        help='The sample based planner to run on the environment.')
    parser.add_argument("-z", "--savefig", action="store_true", help="Save the generated plot.")

    parser.add_argument("-n", "--samples", type=int, required=True, help="The number of random samples to use.")
    parser.add_argument("-s", "--steplen", type=float, required=True, help="The step length for the local planner.")

    # PRM options
    parser.add_argument("-x", "--positions", help="A space separated list of tuples containing the start and goal positions.")
    parser.add_argument("-k", "--neighbors", type=int, help="The number of near neighbors to connect (applicable only to the PRM planner).")
    parser.add_argument("-g", "--gaussian", action='store_true', help="Use a Gaussian sampling strategy (applicable only to the PRM planner).")
    parser.add_argument("-v", "--variance", type=float, help="The variance to use for each Gaussian sample (applicable only to the PRM planner).")

    # RRT options
    parser.add_argument("-b", "--bias", type=float, help="The bias probability (applicable only to RRT planner).")
    parser.add_argument("-c", "--rrtconnect", action='store_true', help="Use an RRT Connect strategy (applicable only to RRT planner).")
    parser.add_argument("-d", "--bidirectional", action='store_true', help="Use an Bi-directional RRT Connect strategy (applicable only to RRT planner).")

    args = parser.parse_args()

    env_file = args.environment
    if env_file == "env0.txt":
        env = "env0"
    else:
        env = "env1"

    # Create the PolygonEnvironment
    poly_env = collisions.PolygonEnvironment()
    poly_env.read_env(env_file)

    if isinstance(poly_env.robot, collisions.RectRobot):
        n = 2
    else:
        n = 3

    N = args.samples
    epsilon = args.steplen

    if args.planner == 'prm':

        if args.gaussian:
            planner = prm.PRM(num_samples=N, num_neighbors=args.neighbors, num_dimensions=n,
                              step_length=epsilon, lims=poly_env.lims, gaussian=True, variance=args.variance,
                              collision_func=poly_env.test_collisions)

            dir = "./figures/prm_gaussian/{0}/".format(env)
            filename = "{0}-{1}-{2}-{3}".format(N, args.neighbors, epsilon, args.variance)
        else:
            planner = prm.PRM(num_samples=N, num_neighbors=args.neighbors, num_dimensions=n,
                              step_length=epsilon, lims=poly_env.lims, collision_func=poly_env.test_collisions)

            dir = "./figures/prm/{0}/".format(env)
            filename = "{0}-{1}-{2}".format(N, args.neighbors, epsilon)

        original_roadmap = planner.build_prm()

        positions = args.positions.split()
        for pos in positions:

            poly_env.start = ast.literal_eval(pos)[0]
            poly_env.goal = ast.literal_eval(pos)[1]

            planner.T = original_roadmap
            poly_env.draw_plan(None, planner)

            # Copy the original roadmap and plan over that copy
            roadmap = copy.deepcopy(original_roadmap)
            plan = planner.find_plan(poly_env.start, poly_env.goal, roadmap)
            planner.T = roadmap

            # Draw and save the environment and the plan (if one exists)
            poly_env.draw_plan(plan, planner)

            if args.savefig:
                savefig(dir, filename)
            plotter.close()
    else:
        planner = rrt.RRT(num_samples=N, num_dimensions=n, step_length=epsilon, lims=poly_env.lims,
                          connect_prob=args.bias, collision_func=poly_env.test_collisions)

        filename = "{0}-{1}-{2}".format(N, epsilon, args.bias)

        if args.rrtconnect:
            plan = planner.build_rrt_connect(poly_env.start, poly_env.goal)
            dir = "./figures/rrt_connect/{0}/".format(env)
        elif args.bidirectional:
            plan = planner.build_bidirectional_rrt(poly_env.start, poly_env.goal)
            dir = "./figures/bidirectional_rrt/{0}/".format(env)
        else:
            plan = planner.build_rrt(poly_env.start, poly_env.goal)
            dir = "./figures/rrt/{0}/".format(env)

        # Draw and save the environment and the plan (if one exists)
        poly_env.draw_plan(plan, planner, show=False)

        if args.savefig:
            savefig(dir, filename)
