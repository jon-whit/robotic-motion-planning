File Descriptions
-----------------

main.py - Main application entry point (see usage below).
rrt.py - Package providing helper classes and functions for constructing and planning over RRTs.
prm.py - Package providing helper classes and functions for constructing and planning over PRMs.
collisions.py - Package providing helper classes and functions for the environments and robots used in this assignment.
Proj2-answers.pdf - A PDF file containing the responses for the questions asked in the project handout.

Requirements
------------
* Python 2.7.11
* numpy >= 1.10.0
* matplotlib >= 1.5.0
* scipy >= 0.18.1


Command Line Interface Usage Guide
----------------------------------

usage: main.py [-h] -e ENVIRONMENT -p {prm,rrt} [-z] -n SAMPLES -s STEPLEN
               [-x POSITIONS] [-k NEIGHBORS] [-g] [-v VARIANCE] [-b BIAS] [-c]
               [-d]

optional arguments:
  -h, --help            show this help message and exit
  -e ENVIRONMENT, --environment ENVIRONMENT
                        The txt file defining the environment.
  -p {prm,rrt}, --planner {prm,rrt}
                        The sample based planner to run on the environment.
  -z, --savefig         Save the generated plot.
  -n SAMPLES, --samples SAMPLES
                        The number of random samples to use.
  -s STEPLEN, --steplen STEPLEN
                        The step length for the local planner.
  -x POSITIONS, --positions POSITIONS
                        A space separated list of tuples containing the start
                        and goal positions.
  -k NEIGHBORS, --neighbors NEIGHBORS
                        The number of near neighbors to connect (applicable
                        only to the PRM planner).
  -g, --gaussian        Use a Gaussian sampling strategy (applicable only to
                        the PRM planner).
  -v VARIANCE, --variance VARIANCE
                        The variance to use for each Gaussian sample
                        (applicable only to the PRM planner).
  -b BIAS, --bias BIAS  The bias probability (applicable only to RRT planner).
  -c, --rrtconnect      Use an RRT Connect strategy (applicable only to RRT
                        planner).
  -d, --bidirectional   Use an Bi-directional RRT Connect strategy (applicable
                        only to RRT planner).
                        

The command line interface (CLI) can be used to specify the specific algorithms that should be run. Below are some examples of how to use the CLI:

* Standard PRM with N=250, K=5, epsilon=2, with multiple starting and goal positions
main.py -e env0.txt -p prm -n 250 -k 5 -s 2 -x "((-50,50),(75,80)),((-4,0),(25,120))"

* Gaussian PRM with the same parameters above and a variance of 12
main.py -e env0.txt -p prm -n 250 -k 5 -s 2 -v 12 -g -x "((-50,50),(75,80)),((-4,0),(25,120))"

* Standard RRT with N=750, epsilon=2, bias=5%
main.py -e env0.txt -p rrt -n 750 -s 2 -b 0.05

* RRT-Connect with the same parameters as the standard RRT above
main.py -e env0.txt -p rrt -c -n 750 -s 2 -b 0.05

* Bidirectional RRT-Connect with the same parameters as above
main.py -e env0.txt -p rrt -d -n 750 -s 2 -b 0.05

If help is needed with the CLI, feel free to email me for further information. It should be quite intuitive.
jon.whitaker@utah.edu