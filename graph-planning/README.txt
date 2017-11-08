File Descriptions
-----------------

main.py - Main application entry point (see usage below).
graph_search.py - Package providing helper classes and functions for performing graph search operations for planning.
Proj1-answers.pdf - A PDF file containing the responses for the questions asked in the project handout.

Requirements
------------
* Python 2.7.11
* numpy >= 1.10.0
* matplotlib >= 1.5.0


Command Line Interface Usage Guide
----------------------------------

usage: main.py [-h] -i INPUT [-x] [-d] -a {dfs,bfs,iddfs,ucs,astar}
               [{dfs,bfs,iddfs,ucs,astar} ...]
               [--heuristic [{euclidean,manhattan}]]

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        The input map file path.
  -x, --disp_path       Print and plot the results.
  -d, --diagonals       Use diagonal actions.
  -a {dfs,bfs,iddfs,ucs,astar} [{dfs,bfs,iddfs,ucs,astar} ...], --algorithms {dfs,bfs,iddfs,ucs,astar} [{dfs,bfs,iddfs,ucs,astar} ...]
                        1 or more path finding algorithms to run.
  --heuristic [{euclidean,manhattan}]
                        The heuristic which will be used for A*.
                        

The command line interface can be used to specify the specific algorithms that should be run, and the actions or/and heuristic which should be applied for the given algorithm(s). To run more than one path finding algorithm, specify them all at the command line using the '-a' flag. For example, to run and plot the results of Depth-first Search, Uniform Cost Search, and A* with a Euclidean distance heuristic on map0.txt, you would use the command:

python main.py -i map0.txt -a dfs ucs astar --heuristic euclidean -x