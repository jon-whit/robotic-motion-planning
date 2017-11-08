File Descriptions
-----------------

main.py - Main application entry point (see usage below).
mdp.py - Package providing helper classes and functions for generating policies via MDP techniques (Value Iteration and Policy Iteration).
gridmap.py - Package providing helper classes and functions for constructing and plotting the grid environments.
Proj3-answers.pdf - A PDF file containing the responses for the questions asked in the project handout.

Requirements
------------
* Python 2.7.11
* numpy >= 1.10.0
* matplotlib >= 1.5.0
* scipy >= 0.18.1


Command Line Interface Usage Guide
----------------------------------
usage: main.py [-h] -i INPUT [-d] -p PROBS -a {mdp,vi,pi} [--absorb]
               [--rewards REWARDS] [--discount DISCOUNT] [--epsilon EPSILON]
               [--policy POLICY] [--random_update]

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        The environment map file path.
  -d, --diagonals       Use diagonal actions in addition to the four cardinal
                        directions.
  -p PROBS, --probs PROBS
                        A comma delimited list of probabilities
                        (command,neighboring,orthogonal).
  -a {mdp,vi,pi}, --algorithm {mdp,vi,pi}
                        The algorithm which should be run.
  --absorb              Sets the goal as an absorbing state, so no transitions
                        are computed over it.
  --rewards REWARDS     A comma delimited list of rewards (applies to VI and
                        PI only).
  --discount DISCOUNT   The discount factor to use (applies to VI and PI
                        only).
  --epsilon EPSILON     The epsilon to converge to.
  --policy POLICY       The policy to use for each state.
  --random_update       Break ties by randomizing the action (defaults to False).                        

The command line interface (CLI) can be used to specify the specific algorithms that should be run. Below are some examples of how to use the CLI:

* To run MDP on map0.txt with an 80% commanded probability, 0% neighboring, and 10% orthogonal you would run the command:
python main.py -i map0.txt -p 0.8,0,0.1 -a mdp

* To run VI on map1.txt for problem 2.4 you would run the command:
python main.py -i map1.txt -d -a vi -p 0.7,0.1,0.05 --rewards 10,0,0 --discount 0.8 --epsilon 0.001

* To run PI on map0.txt for problem 3.1 you would run the command:
python main.py -i map0.txt -a vi -p 0.8,0,0.1 --rewards 10,0,0 --discount 0.8 --epsilon 0.001 --policy u

If help is needed with the CLI, feel free to email me for further information. It should be quite intuitive.
jon.whitaker@utah.edu