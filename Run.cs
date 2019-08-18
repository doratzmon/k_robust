using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Accord.Statistics;

namespace CPF_experiment
{

    /// <summary>
    /// This class is responsible for running the experiments.
    /// </summary>
    public class Run : IDisposable
    {
        ////////debug
        // public static TextWriter resultsWriterdd;
        /////////////

        /// <summary>
        /// Delimiter character used when writing the results of the runs to the output file.
        /// </summary>
        /// 
        public static bool toPrint = false;

        public static int upperB = 2;

        public static readonly string RESULTS_DELIMITER = ",";

        public static readonly int SUCCESS_CODE = 1;

        public static readonly int FAILURE_CODE = 0;

        public static readonly bool WRITE_LOG = false;
        /// <summary>
        /// Number of random steps performed when generating a new problem instance for choosing a start-goal pair.
        /// </summary>
        public static int RANDOM_WALK_STEPS = 100000;

        /// <summary>
        /// Indicates the starting time in ms for timing the different algorithms.
        /// </summary>
        private double startTime;

        public Plan plan;

        public  int solutionCost = -1;
        //Reasonable 
        public enum ConstraintPolicy { Single, Range, DoubleRange };

        /// <summary>
        /// This hold an open stream to the results file.
        /// </summary>
        private TextWriter resultsWriter;

        /// <summary>
        /// EH: I introduced this variable so that debugging and experiments
        /// can have deterministic results.
        /// </summary>
        public static Random rand = new Random();

        /// <summary>
        /// Calls resultsWriter.Dispose()
        /// </summary>
        protected virtual void Dispose(bool dispose_managed)
        {
            if (dispose_managed)
            {
                if (this.resultsWriter != null)
                {
                    this.resultsWriter.Dispose();
                    this.resultsWriter = null;
                }
            }
        }

        public void Dispose() {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Open the results file for output. Currently the file is opened in append mode.
        /// </summary>
        /// <param name="fileName">The name of the results file</param>
        public void OpenResultsFile(string fileName)
        {
            this.resultsWriter = new StreamWriter(fileName, true); // 2nd argument indicates the "append" mode
        }

        /// <summary>
        /// Closes the results file.
        /// </summary>
        public void CloseResultsFile()
        {
            this.resultsWriter.Close();
        }

        /// <summary>
        /// all types of algorithms to be run
        /// </summary>
        List<ISolver> solvers;

        /// <summary>
        /// all types of heuristics used
        /// </summary>
        public List<HeuristicCalculator> heuristics; // FIXME: Make unpublic again later

        /// <summary>
        /// Counts the number of times each algorithm went out of time consecutively
        /// </summary>
        public int[] outOfTimeCounters;

        /// <summary>
        /// Construct with chosen algorithms.
        /// </summary>
        public Run()
        {
            this.watch = Stopwatch.StartNew();

            // Preparing the heuristics:
            heuristics = new List<HeuristicCalculator>();
            var sic = new SumIndividualCosts();
            heuristics.Add(sic);
            var astar = new ClassicAStar(sic, false, false, 0);             //withNoBias
            var astarWithBias1 = new ClassicAStar(sic, false, false, 1);    //withBias
            var astarWithBias2 = new ClassicAStar(sic, false, false, 2);    //withBias2
            var astarWithBias3 = new ClassicAStar(sic, false, false, 3);    //withBias3
            var astarWithBias4 = new ClassicAStar(sic, false, false, 4);    //withBias 4
            var astarWithBias5 = new ClassicAStar(sic, false, false, 5);    //withBias 5
            var astarWithBias6 = new ClassicAStar(sic, false, false, 6);    //withBias 6
            var astarWithBias7 = new ClassicAStar(sic, false, false, 7);    //withBias 7
            var astarWithDynamicBias = new ClassicAStar(sic, false, false, -1);   //withBias -1
            var cbs = new CBS_LocalConflicts(astar, astar, -1);
            var astar_with_od = new AStarWithOD(sic);
            var epea = new AStarWithPartialExpansion(sic);
            var macbsLocal5Epea = new CBS_LocalConflicts(astar, epea, 5);
            var macbsLocal50Epea = new CBS_LocalConflicts(astar, epea, 50);
            

            // Preparing the solvers:
            solvers = new List<ISolver>();
            
             //k-robust 0,1,2, .. , 7
             
            solvers.Add(new CBS_GlobalConflicts(astar, epea, -1, false, CBS_LocalConflicts.BypassStrategy.BEST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 0, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
         
            solvers.Add(new CBS_GlobalConflicts(astarWithBias1, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 1, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
            
            solvers.Add(new CBS_GlobalConflicts(astarWithBias2, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 2, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
            /*
            solvers.Add(new CBS_GlobalConflicts(astarWithBias3, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 3, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias4, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 4, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias5, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 5, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias6, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 6, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1

            solvers.Add(new CBS_GlobalConflicts(astarWithBias7, epea, -1, false, CBS_LocalConflicts.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                        false, CBS_LocalConflicts.ConflictChoice.FIRST, false, false, 1, false, 7, ConstraintPolicy.Range)); // CBS/EPEA* + CARDINAL + BP1
*/
            outOfTimeCounters = new int[solvers.Count];
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        /// <summary>
        /// Generates a problem instance, including a board, start and goal locations of desired number of agents
        /// and desired precentage of obstacles
        /// TODO: Refactor to use operators.
        /// </summary>
        /// <param name="gridSize"></param>
        /// <param name="agentsNum"></param>
        /// <param name="obstaclesNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateProblemInstance(int gridSize, int agentsNum, int obstaclesNum)
        {
            m_mapFileName = "GRID" + gridSize + "X" + gridSize;
            m_agentNum = agentsNum;
            /**
             * Randomization based on timer is disabled for purposes of getting
             * reproducible experiments.
             */
            //Random rand = new Random();

            if (agentsNum + obstaclesNum + 1 > gridSize * gridSize)
                throw new Exception("Not enough room for " + agentsNum + ", " + obstaclesNum + " and one empty space in a " + gridSize + "x" + gridSize + "map.");

            int x;
            int y;
            Agent[] aGoals = new Agent[agentsNum];
            AgentState[] aStart = new AgentState[agentsNum];
            bool[][] grid = new bool[gridSize][];
            bool[][] goals = new bool[gridSize][];

            // Generate a random grid
            for (int i = 0; i < gridSize; i++)
            {
                grid[i] = new bool[gridSize];
                goals[i] = new bool[gridSize];
            }
            for (int i = 0; i < obstaclesNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (grid[x][y]) // Already an obstacle
                    i--;
                grid[x][y] = true;
            }

            // Choose random goal locations
            for (int i = 0; i < agentsNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (goals[x][y] || grid[x][y])
                    i--;
                else
                {
                    goals[x][y] = true;
                    aGoals[i] = new Agent(x, y, i);
                }
            }

            // Select random start/goal locations for every agent by performing a random walk
            for (int i = 0; i < agentsNum; i++)
            {
                aStart[i] = new AgentState(aGoals[i].Goal.x, aGoals[i].Goal.y, aGoals[i]);
            }

            // Initialized here only for the IsValid() call. TODO: Think how this can be sidestepped elegantly.
            ProblemInstance problem = new ProblemInstance();
            problem.Init(aStart, grid);
            
            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = false; // We're going to move the goal somewhere else
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        aStart[i].lastMove.Update(op);
                        if (problem.IsValid(aStart[i].lastMove) &&
                            !goals[aStart[i].lastMove.x][aStart[i].lastMove.y]) // this spot isn't another agent's goal
                            break;
                        else
                            aStart[i].lastMove.setOppositeMove(); // Rollback
                    }
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in aStart) 
            {
                agentStart.lastMove.time = 0;
            }

            // TODO: There is some repetition here of previous instantiation of ProblemInstance. Think how to elegantly bypass this.
            problem = new ProblemInstance();
            problem.Init(aStart, grid);
            return problem;            
        }

        public string m_mapFileName = "";
        public int    m_agentNum    = 0;

        /// <summary>
        /// Generates a problem instance based on a DAO map file.
        /// TODO: Fix code dup with GenerateProblemInstance and Import later.
        /// </summary>
        /// <param name="agentsNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateDragonAgeProblemInstance(string mapFileName, int agentsNum)
        {
            /**
             * Randomization based on timer is disabled for purposes of getting
             * reproducible experiments.
             */
            //Random rand = new Random();

            m_mapFileName       = mapFileName;
            m_agentNum          = agentsNum; 
            TextReader input    = new StreamReader(mapFileName);
            string[] lineParts;
            string line;

            line = input.ReadLine();
            Debug.Assert(line.StartsWith("type octile"));

            // Read grid dimensions
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Debug.Assert(lineParts[0].StartsWith("height"));
            int maxX = int.Parse(lineParts[1]);
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Debug.Assert(lineParts[0].StartsWith("width"));
            int maxY = int.Parse(lineParts[1]);
            line = input.ReadLine();
            Debug.Assert(line.StartsWith("map"));
            bool[][] grid = new bool[maxX][];
            char cell;
            for (int i = 0; i < maxX; i++)
            {
                grid[i] = new bool[maxY];
                line = input.ReadLine();
                for (int j = 0; j < maxY; j++)
                {
                    cell = line.ElementAt(j);
                    if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                        grid[i][j] = true;
                    else
                        grid[i][j] = false;
                }
            }

            int x;
            int y;
            Agent[] agentGoals = new Agent[agentsNum];
            AgentState[] agentStates = new AgentState[agentsNum];
            bool[][] goals = new bool[maxX][];

            for (int i = 0; i < maxX; i++)
                goals[i] = new bool[maxY];

            // Choose random valid unclaimed goal locations
            for (int i = 0; i < agentsNum; i++)
            {
                x = rand.Next(maxX);
                y = rand.Next(maxY);
                if (goals[x][y] || grid[x][y])
                    i--;
                else
                {
                    goals[x][y] = true;
                    agentGoals[i] = new Agent(x, y, i);
                }
            }

            // Select random start/goal locations for every agent by performing a random walk
            for (int i = 0; i < agentsNum; i++)
            {
                agentStates[i] = new AgentState(agentGoals[i].Goal.x, agentGoals[i].Goal.y, agentGoals[i]);
            }

            ProblemInstance problem = new ProblemInstance();
            problem.parameters[ProblemInstance.GRID_NAME_KEY] = Path.GetFileNameWithoutExtension(mapFileName);
            problem.Init(agentStates, grid);

            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = false; // We're going to move the goal somewhere else
                    // Move in a random legal direction:
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        agentStates[i].lastMove.Update(op);
                        if (problem.IsValid(agentStates[i].lastMove) &&
                            !goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y]) // This spot isn't another agent's goal
                            break;
                        else
                            agentStates[i].lastMove.setOppositeMove(); // Rollback
                    }
                    goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in agentStates)
                agentStart.lastMove.time = 0;
            
            return problem;
        }

        
        public static double    planningTime;
        public static int       instanceId = 0; 

        /// <summary>
        /// Solve given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to solve</param>
        public bool SolveGivenProblem(ProblemInstance instance, bool toExecute = false)
        {
            if (toExecute)
            {
                instanceId += 1;
            }
            bool success = true;

            //return; // add for generator
            // Preparing a list of agent indices (not agent nums) for the heuristics' Init() method
            List<uint> agentList = Enumerable.Range(0, instance.m_vAgents.Length).Select<int, uint>(x=> (uint)x).ToList<uint>(); // FIXME: Must the heuristics really receive a list of uints?
            
            // Solve using the different algorithms
            Debug.WriteLine("Solving " + instance);

            //this.PrintProblemStatistics(instance);

            //double cr0 = instance.getConflictRation(0);
            //double cr1 = instance.getConflictRation(1);

            //Debug.WriteLine("Conflict ratio (first order): " + cr0);


            // Initializing all heuristics, whereever they're used
            for (int i = 0; i < heuristics.Count; i++)
                heuristics[i].init(instance, agentList);

            solutionCost = -1;
            int firstSolverToSolveIndex = -1;
            for (int i = 0; i < solvers.Count; i++)
            {
                //CbsNode.PRobustBound = 0.4;
                //CbsNode.PRobustBound = PRobustBound[i];
                if (outOfTimeCounters[i] < Constants.MAX_FAIL_COUNT) // After "MAX_FAIL_COUNT" consecutive failures of a given algorithm we stop running it.
                                                                    // Assuming problem difficulties are non-decreasing, if it consistently failed on several problems it won't suddenly succeed in solving the next problem.
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();

                    if (solvers[i].GetType() == typeof(CBS_LocalConflicts) || solvers[i].GetType() == typeof(CBS_GlobalConflicts))
                    {
                        if (((CBS_LocalConflicts)solvers[i]).mergeThreshold == 314159) // MAGIC NUMBER WHICH MAKES US ADJUST B according to map
                        {
                            ((CBS_LocalConflicts)solvers[i]).mergeThreshold = 0;
                        }
                    }


                    if (
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(CBS_LocalConflicts)) ||
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(CBS_GlobalConflicts))
                       )
                    {
                        if (((CBS_LocalConflicts)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold == 314159) // MAGIC NUMBER SEE ABOVE
                        {
                            ((CBS_LocalConflicts)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold = 0;
                        }
                    }

                    
                    int solverSolutionCost;
                    Stopwatch planningStopwatch = new Stopwatch(); ;
      
                        planningStopwatch.Start();
                        this.run(solvers[i], instance);
                        solverSolutionCost = solvers[i].GetSolutionCost();
         
                        if (solverSolutionCost < 0)
                        {
                            Console.WriteLine("TIMEOUT!!!!!!!!!!!!2");
                            solverSolutionCost = -1;
                            solutionCost = -1;
                            planningStopwatch.Stop();
                            success = false;
                            continue;
                        }
                        if (toPrint)
                        {
                            Console.WriteLine();
                            printLinkedList(solvers[i].GetPlan().GetLocations());
                            Console.WriteLine();
                        }


                    success = true;

                    if (solverSolutionCost >= 0) // Solved successfully
                        {
                            plan = solvers[i].GetPlan();
                            int j = 0;
                            for (LinkedListNode<List<Move>> node = plan.GetLocations().First; node != null; node = node.Next)
                            {
                                foreach (Move mv in node.Value)
                                {
                                    ((TimedMove)mv).time = j;
                                }
                                j++;
                            }

                        

                        outOfTimeCounters[i] = 0;

                            // Validate solution:
                            if (solutionCost == -1) // Record solution cost
                            {
                                solutionCost = solverSolutionCost;
                                firstSolverToSolveIndex = i;
                            }
                            solutionCost = solverSolutionCost;

                            Console.WriteLine("+SUCCESS+ (:");
                        }
                        else
                        {
                            outOfTimeCounters[i]++;
                            Console.WriteLine("-FAILURE- ):");
                            success = false;
                        }

                        int originalCost = solutionCost;
                        planningStopwatch.Stop();
                        planningTime = planningStopwatch.Elapsed.TotalMilliseconds;
                        TimedMove[] lastMove = new TimedMove[instance.m_vAgents.Length];
                        for (int ass = 0; ass < instance.m_vAgents.Length; ass++)
                        {
                            lastMove[ass] = new TimedMove(instance.m_vAgents[ass].lastMove, 0);
                        }

                    WriteGivenProblem(instance, solvers[i], plan.GetLocations());

                }
                else if(toPrint)
                    PrintNullStatistics(solvers[i]);
                
                Console.WriteLine();
            }
            return success;
        }

       

        public double elapsedTime;

        /// <summary>
        /// Solve a given instance with the given solver
        /// </summary>
        /// <param name="solver">The solver</param>
        /// <param name="instance">The problem instance that will be solved</param>
        private void run(ISolver solver, ProblemInstance instance)
        {
            // Run the algorithm
            bool solved;
            Console.WriteLine("-----------------" + solver + "-----------------");
            this.startTime = this.ElapsedMillisecondsTotal();
            solver.Setup(instance, this);
            solved = solver.Solve();
            elapsedTime = this.ElapsedMilliseconds();
            if (solved)
            {
                Console.WriteLine("Total cost: {0}", solver.GetSolutionCost());
                Console.WriteLine("Solution depth: {0}", solver.GetSolutionDepth());
            }
            else
            {
                Console.WriteLine("Failed to solve");
                Console.WriteLine("Solution depth lower bound: {0}", solver.GetSolutionDepth());
            }
            Console.WriteLine();
            Console.WriteLine("Time In milliseconds: {0}", elapsedTime);
           // Console.WriteLine("Total Unique/Full Expanded Nodes: {0}", solver.GetNodesPassedPruningCounter());
            if(toPrint)
                this.PrintStatistics(instance, solver, elapsedTime);
            // Solver clears itself when it finishes the search.
            solver.ClearStatistics();
        }

        /// <summary>
        /// Print the header of the results file
        /// </summary>
        public void PrintResultsFileHeader()
        {
            this.resultsWriter.Write("Grid Name");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Rows");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Columns");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Agents");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Obstacles");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Instance Id");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);

            for (int i = 0; i < solvers.Count; i++)
            {
                var solver = solvers[i];
                this.resultsWriter.Write(solver + " Success");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Runtime");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Solution Cost");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                solver.OutputStatisticsHeader(this.resultsWriter);
                this.resultsWriter.Write(solver + " Max Group");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Solution Depth");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);

                //this.resultsWriter.Write(name + "Min Group / G&D");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Max depth");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Memory Used");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            }
            this.ContinueToNextLine();
        }

        /// <summary>
        /// Print the solver statistics to the results file.
        /// </summary>
        /// <param name="instance">The problem instance that was solved. Not used!</param>
        /// <param name="solver">The solver that solved the problem instance</param>
        /// <param name="runtimeInMillis">The time it took the given solver to solve the given instance</param>
        private void PrintStatistics(ProblemInstance instance, ISolver solver, double runtimeInMillis)
        {
            // Success col:
            if (solver.GetSolutionCost() < 0)
                this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(Run.SUCCESS_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(runtimeInMillis + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write(solver.GetSolutionCost() + RESULTS_DELIMITER);
            // Algorithm specific cols:
            solver.OutputStatistics(this.resultsWriter);
            // Max Group col:
            this.resultsWriter.Write(solver.GetMaxGroupSize() + RESULTS_DELIMITER);
            // Solution Depth col:
            this.resultsWriter.Write(solver.GetSolutionDepth() + RESULTS_DELIMITER);
            //this.resultsWriter.Flush();
        }

        private void PrintProblemStatistics(ProblemInstance instance)
        {
            // Grid Name col:
            if (instance.parameters.ContainsKey(ProblemInstance.GRID_NAME_KEY))
                this.resultsWriter.Write(instance.parameters[ProblemInstance.GRID_NAME_KEY] + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(RESULTS_DELIMITER);
            // Grid Rows col:
            this.resultsWriter.Write(instance.m_vGrid.Length + RESULTS_DELIMITER);
            // Grid Columns col:
            this.resultsWriter.Write(instance.m_vGrid[0].Length + RESULTS_DELIMITER);
            // Num Of Agents col:
            this.resultsWriter.Write(instance.m_vAgents.Length + RESULTS_DELIMITER);
            // Num Of Obstacles col:
            this.resultsWriter.Write(instance.m_nObstacles + RESULTS_DELIMITER);
            // Instance Id col:
            this.resultsWriter.Write(instance.instanceId + RESULTS_DELIMITER);
        }

        private void ContinueToNextLine()
        {
            this.resultsWriter.WriteLine();
            this.resultsWriter.Flush();
        }

        private void PrintNullStatistics(ISolver solver)
        {
            // Success col:
            this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(Constants.MAX_TIME + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Algorithm specific cols:
            for (int i = 0; i < solver.NumStatsColumns; ++i)
                this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Max Group col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Solution Depth col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
        }
        
        public void ResetOutOfTimeCounters()
        {
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        private Stopwatch watch;
        private double ElapsedMillisecondsTotal()
        {
            return this.watch.Elapsed.TotalMilliseconds;
        }

        public double ElapsedMilliseconds()
        {
            return ElapsedMillisecondsTotal() - this.startTime;
        }

        public void StartOracle()
        {
            this.watch.Stop();
            // NOTE: This allows the algorithm with the oracle solve harder problems without timing out, getting
            // a higher average timeout than running without the oracle, which isn't what we want.
            // We need to start another counter when the oracle runs and when the run successfully finishes
            // substract its count
        }

        public void StopOracle()
        {
            this.watch.Start();
        }

        public static double TIMEOUT    = 300000;
        //public static double TIMEOUT = double.MaxValue;


        /// <summary>
        /// Write to file a given instance 
        /// </summary>
        /// <param name="instance">The instance to execute</param>
        public void WriteGivenProblem
        (
            ProblemInstance instance, 
            ISolver solver, 
            LinkedList<List<Move>> currentPlan = null)
        {
            int solutionSumOfCost;

            if (currentPlan == null)
            {
                currentPlan = copyLinkedList(plan.GetLocations());
                solutionSumOfCost = solutionCost;
                
            }
            else
                solutionSumOfCost = computeSumOfCost(currentPlan);
            double planTime = planningTime;
              
            GC.Collect();
            GC.WaitForPendingFinalizers();
            string solverName = "";
            if (solver.GetType() == typeof(CBS_LocalConflicts) || solver.GetType() == typeof(CBS_GlobalConflicts))
                solverName = "ICBS + " + ((CBS_LocalConflicts)solver).conflictRange;
            else
                solverName = "EPEA*";



            writeToFile(
                solverName,                             // solver name
                planTime.ToString(),                    // planning time             
                solutionSumOfCost.ToString(),           // solution cost
                instanceId.ToString(),                  // instanceId  
                instance.fileName,                      // file Name
                instance.m_vAgents.Length.ToString(),   // #Agents
                m_mapFileName,                          // Map name
                instance.m_nObstacles,                  // Obstacles
                ((CBS_GlobalConflicts)solver).constraintPolicy);  // Constraint Policy           
        }
        


        /// <summary>
        /// write execution info to file
        /// </summary>
        public void writeToFile
        (
            string solver, 
            string planTime, 
            string planCost, 
            string instanceId, 
            string instanceName,
            string agentsCount, 
            string mapFileName, 
            uint obstaclesPercents, 
            ConstraintPolicy constraintPolicy
        )
        {
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustCSV.csv";
            int length;
            string delimter = ",";
            List<string[]> output = new List<string[]>();
            string[] temps = new string[10];


            if (!File.Exists(filePath))
            {
                File.Create(filePath).Close();
                
                temps[0] = "IncstanceId";
                temps[1] = "#Agents";
                temps[2] = "Solver";
                temps[3] = "Map";
                temps[4] = "Obstacles";
                temps[5] = "Plan Cost";
                temps[6] = "Success";
                temps[7] = "Plan time";
                temps[8] = "Constraint policy";
                temps[9] = "File";
                output.Add(temps);

                length = output.Count;
                using (System.IO.TextWriter writer = File.AppendText(filePath))
                {
                    for (int index = 0; index < length; index++)
                    {
                        writer.WriteLine(string.Join(delimter, output[index]));
                    }
                }
            }
            string new_file_name = instanceName;
            string[] splitFilePath = new_file_name.Split('\\');
            new_file_name = splitFilePath[splitFilePath.Count() - 1];
            string map = (new_file_name.Split('.'))[0];
            string instance = (new_file_name.Split('-'))[1];

            //temps[0] = instanceId;
            temps[0] = instance;
            temps[1] = agentsCount;
            temps[2] = solver;
            //temps[3] = Path.GetFileNameWithoutExtension(mapFileName);
            temps[3] = map;
            temps[4] = obstaclesPercents.ToString(); ;
            temps[5] = planCost;
            if (temps[5].Equals("-1") || temps[5].Equals("-2") || temps[5].Equals("-3"))
            {
                temps[6] = "0";
            }
            else
                temps[6] = "1";
            temps[7] = planTime;
            temps[8] = constraintPolicyToString(constraintPolicy);
            
            temps[9] = new_file_name;

            output.Add(temps);

            length = output.Count;
            using (System.IO.TextWriter writer = File.AppendText(filePath))
            {
                for (int index = 0; index < length; index++)
                {
                    writer.WriteLine(string.Join(delimter, output[index]));
                }
            }
        }


        /// <summary>
        /// convert constraint policy to string
        /// </summary>
        private string constraintPolicyToString(ConstraintPolicy policy)
        {
            if (policy == ConstraintPolicy.Single)
                return "Single";
            else if (policy == ConstraintPolicy.Range)
                return "Range";
            else if (policy == ConstraintPolicy.DoubleRange)
                return "Double Range";
            else
                return "NO_POLICY";
        }

        /// <summary>
        /// compute sum of cost for a given plan
        /// </summary>
        public int computeSumOfCost(LinkedList<List<Move>> newPlan)// Run runner, ProblemInstance instance)
        {
            int agentNumber = newPlan.ElementAt<List<Move>>(0).Count;//runner.plan.GetLocations().ElementAt<List<Move>>(0).Count;
            int totalCost = 0;
            int currentCounter = 0;
            int currentCost = 0;
            Move previousMove = null;
            for (int i = 0; i < agentNumber; i++)
            {
                currentCounter = 0;
                currentCost = 0;
                foreach (List<Move> locationsAtTime in newPlan)//runner.plan.GetLocations())
                {
                    Move currentMove = locationsAtTime[i];
                    if (currentCounter != 0 && (currentMove.x != previousMove.x || currentMove.y != previousMove.y))
                        currentCost = currentCounter;
                    previousMove = locationsAtTime[i];
                    currentCounter++;
                }
                totalCost += currentCost;
            }
            return totalCost;
        }

        public static Stopwatch replanStopwath;


        private bool checkValidBiasPlan(LinkedList<List<Move>> plan, int bias)
        {
            LinkedListNode<List<Move>> node = plan.First;
            LinkedListNode<List<Move>> biasnode;
            List<Move> biasMove;
            List<Move> cur = node.Value;
            bool valid = true;
            for (int i = 0; i < plan.Count; i++)
            {

                for (int j = 0; j < bias + 1; j++)
                {
                    biasnode = node;
                    for (int p = 0; p < j && biasnode.Next != null; p++)
                    {
                        biasnode = biasnode.Next;
                    }
                    biasMove = biasnode.Value;
                    for (int aMove = 0; aMove < plan.First.Value.Count; aMove++)
                    {
                        for (int bMove = 0; bMove < plan.First.Value.Count; bMove++)
                        {
                            if (aMove == bMove)
                                continue;
                            Move agent1 = cur[aMove];
                            Move agent2 = biasMove[bMove];
                            if (agent1.IsColliding(agent2))
                            {
                                if (toPrint)
                                    Console.WriteLine("Agents " + aMove + " and " + bMove + " collides at time " + i + " bias " + j);
                                valid = false;
                            }
                        }
                    }
                }
                node = node.Next;
                if (node != null)
                    cur  = node.Value;
            }
            if (valid)
            {
                if (toPrint)
                    Console.WriteLine("A valid bias plan!");
            }
            else
            {
                //if (toPrint)
                    Console.WriteLine("Not a valid bias plan!");
            }
            return valid;
        }



        private void correctAgentsCollision(List<Move> preList, ref List<Move> colList/*, ref List<int> addAgentsToDelay*/)
        {
            bool hasCollision = false;
            for (int i = 0; i < colList.Count; i++)
            {
                for (int j = i + 1; j < colList.Count; j++)
                {
                    Move aMove = colList[i];
                    Move bMove = colList[j];
                    if (aMove.IsColliding(bMove)) 
                    {

                        if (aMove.x == preList[i].x && aMove.y == preList[i].y)
                        {
                            colList[j] = new Move(preList[j]);
                        }
                        else if (bMove.x == preList[j].x && bMove.y == preList[j].y)
                        { 
                        colList[i] = new Move(preList[i]);
                        }
                    else
                    {
                        colList[j] = new Move(preList[j]);
                        colList[i] = new Move(preList[i]);
                    }
                        hasCollision = true;
                    }
                }
            }
            if (hasCollision)
            {
                correctAgentsCollision(preList, ref colList);
                return ;
            }
        }


        static int tableWidth = 200;


        private void printLinkedList(LinkedList<List<Move>> toPrint, bool writeToFile = false)
        {
            if (toPrint.Count == 0)
                return;
            PrintLine();
            LinkedListNode<List<Move>> node = toPrint.First;
            string[] columns = new string[node.Value.Count + 1];
            columns[0] = "";
            for(int agentNumber = 1; agentNumber < node.Value.Count + 1; agentNumber++)
            {
                columns[agentNumber] = (agentNumber-1).ToString();

            }
            node = toPrint.First;
            PrintRow(columns);
            PrintLine();
            
            int time = 0;
            while (node != null)
            {
                columns = new string[node.Value.Count + 1];
                columns[0] = time.ToString();
                time++;
                List<Move> currentMoves = node.Value;
                for (int i = 0; i < currentMoves.Count; i++)
                { 
                    Move currentMove = currentMoves[i];
                    columns[i + 1] = currentMove.x + "," + currentMove.y;
                }
                PrintRow(columns);
                node = node.Next;
            }
            PrintLine();
        }

        static void PrintLine()
        {
                Console.WriteLine(new string('-', tableWidth));
        }

        static void PrintRow(params string[] columns)
        {
            int width = (tableWidth - columns.Length) / columns.Length;
            string row = "|";

            foreach (string column in columns)
            {
                row += AlignCentre(column, width) + "|";
            }

            Console.WriteLine(row);

        }

        static string AlignCentre(string text, int width)
        {
            text = text.Length > width ? text.Substring(0, width - 3) + "..." : text;

            if (string.IsNullOrEmpty(text))
            {
                return new string(' ', width);
            }
            else
            {
                return text.PadRight(width - (width - text.Length) / 2).PadLeft(width);
            }
        }


        private LinkedList<List<Move>> copyLinkedList(LinkedList<List<Move>> toCopy)
        {
            LinkedList<List<Move>> newLinkedList = new LinkedList<List<Move>>();
            LinkedListNode<List<Move>> node = toCopy.First;
            while(node != null)
            {
                List<Move> newList = new List<Move>();
                foreach(Move mv in node.Value)
                {
                    Move newMove = new TimedMove((TimedMove)mv);
                    newList.Add(newMove);
                }
                newLinkedList.AddLast(newList);
                node = node.Next;
            }
            return newLinkedList;
        }


        private void compareLinkedList(LinkedList<List<Move>> lA, LinkedList<List<Move>> lB)
        {
            LinkedListNode<List<Move>> nodeA = lA.First;
            LinkedListNode<List<Move>> nodeB = lB.First;
            List<Move> lAcurrentList;
            List<Move> lBcurrentList;
            int aSize = nodeA.Value.Count;
            int bSize = nodeB.Value.Count;
            do
            {
                if (nodeA != null)
                    lAcurrentList = nodeA.Value;
                else
                    lAcurrentList = null;
                if (nodeB != null)
                    lBcurrentList = nodeB.Value;
                else
                    lBcurrentList = null;
                for (int i = 0; i < aSize && i < bSize; i++)
                {
                    if(lAcurrentList != null && lBcurrentList != null)
                        printTimedMove((TimedMove)lAcurrentList[i], (TimedMove)lBcurrentList[i], i);
                    else if(lAcurrentList == null)
                        printTimedMove(null, (TimedMove)lBcurrentList[i], i);
                    else
                        printTimedMove((TimedMove)lAcurrentList[i], null, i);
                }
                if (nodeA != lA.Last)
                    nodeA = nodeA.Next;
                else
                    nodeA = null;
                if (nodeB != lB.Last)
                    nodeB = nodeB.Next;
                else
                    nodeB = null;
            } while (nodeA != lA.Last && nodeB != lB.Last);
        }

        private void printTimedMove(TimedMove mv1, TimedMove mv2, int Agent)
        {
            if(mv1 == null)
                Console.WriteLine("Agent: " + Agent + ", At A: null, At B: " + timedMoveString(mv2));
            else if(mv2 == null)
                Console.WriteLine("Agent: " + Agent + ", At A: " + timedMoveString(mv1) + ", At B: null");
            else if(!compareTimedMoves(mv1, mv2))
                Console.WriteLine("Agent: " + Agent + ", At A: " + timedMoveString(mv1) + ", At B: " + timedMoveString(mv2));
        }

        private string timedMoveString(TimedMove mv)
        {
            return "[ (" + mv.x + "," + mv.y + ") - " + mv.time + " ]";
        }

        private bool compareTimedMoves(TimedMove A, TimedMove B)
        {
            if (A.x == B.x && A.y == B.y && A.time == B.time)
                return true;
            return false;
        }

    }
}
