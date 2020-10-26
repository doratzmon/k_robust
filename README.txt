k-Robust CBS Algorithm with A* as a low level solver
---------------------------------------------------



------  Important classes and methods ------

-- Program class  (The main class of the project)
In the Main method you choose which experiments to run (DragonAge/Grids/Maze/Specific Instance) 
from the folder "/bin/Debug(or Release)/instances/". If the folder doesn't contains an instance, a new
instance will be created.

-- Run class (The class that runs the experiments and generates new instances)
In the Run method you choose which solver to use (CBS)

-- ProblemInstance class (The class that holds the information about the instance)
In the Import method we get an instance name, read the file of the instance, and convert it into data structures 

-- CBS class (The class of the CBS algorithm)
In the Setup and Solve classes we setup the root of the constraint tree and solve the problem by performing a BFS on the tree.

-- CbsNode class (The class of a CBS node)
In the Solve method we find an optimal plan for each agent and identify the conflicts
In the Replan method we replan for the constrained agent 

-- ClassicAStar class (The class of the A* algorithm)
In the solve method we find an optimal path for an agent with respect to the constraints



------  The sequence of the program ------

1. Program.Main creates a new experiment and calls each instance with ProblemInstance.Import

2. The imported instance is given to Run.SolveGivenProblem, which calls Run.run

3. Run.run calls CBS.Setup in order to initialize the CBS root (CbsNode)

4. CbsNode.Solve finds path for each agent with ClassicAStar.Solve and identifies the conflicts

5. Run.run calls CBS.Solve to perform a BFS on the constraint tree, starting from the root

6. Run.SolveGivenProblem prints the solution, if we found one


