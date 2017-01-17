using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// This is the entry point of the application. 
    /// </summary>
    class Program
    {

        public static double replanTime = 0;
        public static int replanCounter = 0;
        private static string RESULTS_FILE_NAME = "Results.csv"; // Overridden by Main
        private static bool onlyReadInstances = false;
        public static double chanceForExecutionMistake = 0.001; //default
        public enum LazyOrEager {Lazy, Eager };
        public static bool TO_EXECUTE = false;

        /// <summary>
        /// Simplest run possible with a randomly generated problem instance.
        /// </summary>
        public void SimpleRun()
        {
            Run runner = new Run();
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            runner.PrintResultsFileHeader();
            ProblemInstance instance = runner.GenerateProblemInstance(10, 3, 10);
            instance.Export("Test.instance");
            runner.SolveGivenProblem(instance, TO_EXECUTE);
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a single instance, imported from a given filename.
        /// </summary>
        /// <param name="fileName"></param>
        public void RunInstance(string fileName)
        {
            ProblemInstance instance;
            try
            {
                instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + fileName);
            }
            catch (Exception e)
            {
                Console.WriteLine(String.Format("Skipping bad problem instance {0}. Error: {1}", fileName, e.Message));
                return;
            }

            Run runner = new Run();
            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();
            runner.SolveGivenProblem(instance, TO_EXECUTE);
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a set of experiments.
        /// This function will generate a random instance (or load it from a file if it was already generated)
        /// </summary>
        public void RunExperimentSet(int[] gridSizes, int[] agentListSizes, int[] obstaclesProbs, int instances)
        {
            writeToFile(true, null, null, null, null, null, null, null, null, LazyOrEager.Lazy, null);

            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();

            bool continueFromLastRun = false;
            string[] LastProblemDetails = null;
            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                var lastProblemFile = new StreamReader(currentProblemFileName);
                LastProblemDetails = lastProblemFile.ReadLine().Split(',');  //get the last problem
                lastProblemFile.Close();
                continueFromLastRun = true;
            }

            for (int gs = 0; gs < gridSizes.Length; gs++)
            {
                for (int obs = 0; obs < obstaclesProbs.Length; obs++)
                {
                    runner.ResetOutOfTimeCounters();
                    for (int ag = 0; ag < agentListSizes.Length; ag++)
                    {
                        if (gridSizes[gs] * gridSizes[gs] * (1 - obstaclesProbs[obs] / 100) < agentListSizes[ag]) // Probably not enough room for all agents
                            continue;
                        for (int i = 0; i < instances; i++)
                        {
                            string allocation = Process.GetCurrentProcess().ProcessName.Substring(1);
                            //if (i % 33 != Convert.ToInt32(allocation)) // grids!
                            //    continue;

                            //if (i % 5 != 0) // grids!
                            //    continue;

                            if (continueFromLastRun)  //set the latest problem
                            {
                                gs = int.Parse(LastProblemDetails[0]);
                                obs = int.Parse(LastProblemDetails[1]);
                                ag = int.Parse(LastProblemDetails[2]);
                                i = int.Parse(LastProblemDetails[3]);
                                for (int j = 4; j < LastProblemDetails.Length; j++)
                                {
                                    runner.outOfTimeCounters[j - 4] = int.Parse(LastProblemDetails[j]);
                                }
                                continueFromLastRun = false;
                                continue; // "current problem" file describes last solved problem, no need to solve it again
                            }
                            if (runner.outOfTimeCounters.Length != 0 &&
                                runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * Constants.MAX_FAIL_COUNT) // All algs should be skipped
                                break;
                            instanceName = "Instance-" + gridSizes[gs] + "-" + obstaclesProbs[obs] + "-" + agentListSizes[ag] + "-" + i;
                            try
                            {
                                instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                                instance.instanceId = i;
                            }
                            catch (Exception importException)
                            {
                                if (onlyReadInstances)
                                {
                                    Console.WriteLine("File " + instanceName + "  dosen't exist");
                                    return;
                                }

                                instance = runner.GenerateProblemInstance(gridSizes[gs], agentListSizes[ag], obstaclesProbs[obs] * gridSizes[gs] * gridSizes[gs] / 100);
                                instance.ComputeSingleAgentShortestPaths(); // REMOVE FOR GENERATOR
                                instance.instanceId = i;
                                instance.Export(instanceName);
                            }

                            runner.SolveGivenProblem(instance, TO_EXECUTE);

                            //return; // DELETE ME!!!!!!!!!!!!!!!!!!!!!!!!!@# JUST FOR DEBUGGING!!!

                            // Save the latest problem
                            if (File.Exists(currentProblemFileName))
                                File.Delete(currentProblemFileName);
                            var lastProblemFile = new StreamWriter(currentProblemFileName);
                            lastProblemFile.Write("{0},{1},{2},{3}", gs, obs, ag, i);
                            for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                            {
                                lastProblemFile.Write("," + runner.outOfTimeCounters[j]);
                            }
                            lastProblemFile.Close();
                        }
                    }
                }
            }
            runner.CloseResultsFile();
        }

        protected static readonly string[] daoMapFilenames = {/* "dao_maps\\den502d.map", "dao_maps\\ost003d.map", */"dao_maps\\brc202d.map" };

        protected static readonly string[] mazeMapFilenames = { "mazes-width1-maps\\maze512-1-6.map", "mazes-width1-maps\\maze512-1-2.map",
                                                "mazes-width1-maps\\maze512-1-9.map" };

        /*private Stopwatch watch = new Stopwatch();
        private double startTime;
        private double ElapsedMillisecondsTotal()
        {
            return this.watch.Elapsed.TotalMilliseconds;
        }

        public double ElapsedMilliseconds()
        {
            return ElapsedMillisecondsTotal() - this.startTime;
        }*/
        public static Stopwatch sw = new Stopwatch();
        /// <summary>
        /// Dragon Age experiment
        /// </summary>
        /// <param name="numInstances"></param>
        /// <param name="mapFileNames"></param>
        public void RunDragonAgeExperimentSet(int numInstances, string[] mapFileNames)
        {
            writeToFile(true, null, null, null, null, null, null, null, null, LazyOrEager.Lazy, null);

            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();
            // FIXME: Code dup with RunExperimentSet

            TextWriter output;
            //int[] agentListSizes = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
            //int[] agentListSizes = { 60, 65, 70, 75, 80, 85, 90, 95, 100 };
            int[] agentListSizes = { /*5, 10, 15, */20 /*, 25, 30, 35, 40, 45, 50*/};

            bool continueFromLastRun = false;
            string[] lineParts = null;

            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                TextReader input = new StreamReader(currentProblemFileName);
                lineParts = input.ReadLine().Split(',');  //get the last problem
                input.Close();
                continueFromLastRun = true;
            }
            double chance = chanceForExecutionMistake;
            //for (double chance = chanceForExecutionMistake; chance < 0.9; chance *= 10)
            //{
            for (int ag = 0; ag < agentListSizes.Length; ag++)
            {
                for (int i = 0; i < numInstances; i++)
                {
                    string name = Process.GetCurrentProcess().ProcessName.Substring(1);
                    //if (i % 33 != Convert.ToInt32(name)) // DAO!
                    //    continue;

                    for (int map = 0; map < mapFileNames.Length; map++)
                    {
                        if (continueFromLastRun) // Set the latest problem
                        {
                            ag = int.Parse(lineParts[0]);
                            i = int.Parse(lineParts[1]);
                            map = int.Parse(lineParts[2]);
                            for (int j = 3; j < lineParts.Length && j - 3 < runner.outOfTimeCounters.Length; j++)
                            {
                                runner.outOfTimeCounters[j - 3] = int.Parse(lineParts[j]);
                            }
                            continueFromLastRun = false;
                            continue;
                        }
                        if (runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * 20) // All algs should be skipped
                            break;
                        string mapFileName = mapFileNames[map];
                        instanceName = Path.GetFileNameWithoutExtension(mapFileName) + "-" + agentListSizes[ag] + "-" + i;
                        try
                        {
                            instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                        }
                        catch (Exception importException)
                        {
                            if (onlyReadInstances)
                            {
                                Console.WriteLine("File " + instanceName + "  dosen't exist");
                                return;
                            }

                            instance = runner.GenerateDragonAgeProblemInstance(mapFileName, agentListSizes[ag]);
                            instance.ComputeSingleAgentShortestPaths(); // Consider just importing the generated problem after exporting it to remove the duplication of this line from Import()
                            instance.instanceId = i;
                            instance.Export(instanceName);
                        }
                        // Run.upperB = 2;
                        replanStopwath = new Stopwatch();
                        replanStopwath.Start();

                        runner.SolveGivenProblem(instance, TO_EXECUTE);

                        replanStopwath.Stop();
                        //save the latest problem
                        File.Delete(currentProblemFileName);
                        output = new StreamWriter(currentProblemFileName);
                        output.Write("{0},{1},{2}", ag, i, map);
                        for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                        {
                            output.Write("," + runner.outOfTimeCounters[j]);
                        }
                        output.Close();
                        /*
                            int newCost;
                            double planTime  = runner.elapsedTime;
                            int solutionCost = runner.solutionCost;
                            

                                Console.WriteLine("***** EXECUTE *****");
                                replanTime = 0;
                                sw = new Stopwatch();
                                sw.Start();
                                replanCounter = 0;
                                LinkedList<List<Move>> newPlan = new LinkedList<List<Move>>();
                                LazyOrEager lazyOrEager = LazyOrEager.Eager;                    
                                string solver           = "ICBS";
                                try
                                {
                                    if (solutionCost > 0)
                                    {
                                        double executionCost = executePlan(runner, instance, newPlan, lazyOrEager, chance);
                                        newCost = computeSumOfCost(newPlan);
                                    }
                                    else
                                        newCost = -1;
                                }
                                catch
                                {
                                   newCost = -1;
                                }
                                sw.Stop();
                                
                                writeToFile(false,
                                    solver,
                                    planTime.ToString(),
                                    sw.Elapsed.TotalMilliseconds.ToString(),
                                    solutionCost.ToString(),
                                    newCost.ToString(),
                                    ag.ToString(),
                                    agentListSizes[ag].ToString(),
                                    runner,
                                    lazyOrEager,
                                    mapFileName);
                            
                        }
                        */
                    }
                }
                //}
                runner.CloseResultsFile();
            }
        }

        public void writeToFile(bool header, string solver, string planTime, string elapsedTime, string planCost, string executionCost, string instanceId, string agentsCount, Run runner, LazyOrEager lazyOrEager, string mapFileName)
        {
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustCSV.csv";

            string delimter = ",";
            List<string[]> output = new List<string[]>();
            string[] temps = new string[12];

            if (header && File.Exists(filePath))
                return;

            if (!File.Exists(filePath))
            {
                File.Create(filePath).Close();
            }
            if (header)
            {
                temps[0] = "IncstanceId";
                temps[1] = "#Agents";
                temps[2] = "Error Prob.";
                temps[3] = "Solver";
                temps[4] = "Policy";
                temps[5] = "Map";
                temps[6] = "Plan Cost";
                temps[7] = "Exec. Cost";
                temps[8] = "Success";
                temps[9] = "#Replans";
                temps[10] = "Plan time";
                temps[11] = "Replan time";
            }
            if (!header)
            {
                temps[0] = instanceId;
                temps[1] = agentsCount;
                temps[2] = chanceForExecutionMistake.ToString();
                temps[3] = solver;
                if (lazyOrEager == LazyOrEager.Lazy)
                    temps[4] = "LAZY";
                else
                    temps[4] = "EAGER";
                temps[5] = Path.GetFileNameWithoutExtension(mapFileName); //brc202d
                temps[6] = planCost;
                temps[7] = executionCost;
                temps[8] = replanCounter.ToString();
                temps[9] = planTime;
                //temps[10] = elapsedTime;
                temps[10] = replanTime.ToString();
            }
            output.Add(temps);

            int length = output.Count;
            using (System.IO.TextWriter writer = File.AppendText(filePath))
            {
                for (int index = 0; index < length; index++)
                {
                    writer.WriteLine(string.Join(delimter, output[index]));
                }
            }
        }

        public int computeSumOfCost(LinkedList<List<Move>> newPlan)// Run runner, ProblemInstance instance)
        {
            int agentNumber = newPlan.ElementAt<List<Move>>(0).Count;//runner.plan.GetLocations().ElementAt<List<Move>>(0).Count;
            int totalCost       = 0;
            int currentCounter  = 0;
            int currentCost     = 0;
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
        public static Stopwatch replanStopwath = new Stopwatch();
        static public Random rand = new Random();
        public int executePlan(Run runner, ProblemInstance instance, LinkedList<List<Move>> newPaths, LazyOrEager lazyOrEager, double chance)
        {
            if (sw.ElapsedMilliseconds > Run.TIMEOUT)
                throw new Exception("TIMEOUT");
            int cost    = 0;
            Move[][] a  = instance.singleAgentOptimalMoves;
            List<Move> previousTime = null;
            List<Move> delayMoves = new List<Move>();
            List<int> delayAgents = new List<int>();
            LinkedList<List<Move>> lLocations = runner.plan.GetLocations();
            int dynamicSize = lLocations.Count;
            LinkedListNode<List<Move>> node = lLocations.First;
            List<Move> locationsAtTime = node.Value;
            for (int i = 0; i < dynamicSize; i++)
            {
                /*if (previousTime != null && checkIfColide(previousTime))
                {
                    Console.WriteLine("PROBLEM6 !!!!!!!");
                    Console.ReadLine();
                }*/
                cost++;
                foreach (Move aMove in locationsAtTime)
                {
                    double randomChance = rand.NextDouble();
                    if (randomChance < chance)
                    {
                        delayMoves.Add((Move)aMove);
                        delayAgents.Add(locationsAtTime.IndexOf((Move)aMove));
                    }
                }
                /*if (previousTime != null && checkIfColide(previousTime))
                {
                    Console.WriteLine("PROBLEM7 !!!!!!!");
                    Console.ReadLine();
                }*/
                if (delayMoves.Count > 0)
                {
                    /*if (previousTime == null)
                    {
                        Console.WriteLine("PROBLEM88 !!!!!!!");
                        Console.ReadLine();
                    }*/
                    agentDelay(delayAgents, delayMoves, runner, previousTime, i);
                   /* if (previousTime == null)
                    {
                        Console.WriteLine("PROBLEM8 !!!!!!!");
                        Console.ReadLine();
                    }*/
                }
                /*if (previousTime!=null && checkIfColide(previousTime))
                {
                    Console.WriteLine("PROBLEM !!!!!!!");
                    Console.ReadLine();
                }*/
                //bool isEager = lazyOrEager == LazyOrEager.Eager;
                if (lazyOrEager == LazyOrEager.Lazy)
                {
                    if (checkIfColide(locationsAtTime))
                    {
                        Console.WriteLine("***** REPLAN *****");
                        replanCounter++;
                        /*if (checkIfColide(previousTime))
                        {
                            if(previousTime == locationsAtTime)
                                Console.WriteLine("asd");
                            Console.WriteLine("immposible!!");
                        }*/
                        if (previousTime == null)
                        {
                            Console.WriteLine("i=" +i);
                            Console.Read();
                        }
                        /*if (checkIfColide(previousTime))
                        {
                            Console.WriteLine("***** PROBLEM4 *****");
                            Console.ReadLine();
                        }*/
                        replanStopwath = new Stopwatch();
                        replanStopwath.Start();
                        try
                        {
                            replan(previousTime, instance, runner);
                        }
                        catch
                        {
                            replanStopwath.Stop();
                            throw new Exception();
                        }
                        replanStopwath.Stop();
                        replanTime += replanStopwath.Elapsed.TotalMilliseconds;
                       /* if (checkIfColide(previousTime))
                        {
                            Console.WriteLine("***** PROBLEM5 *****");
                            Console.ReadLine();
                        }*/
                       /* for (int p = 0; p < previousTime.Count; p++)
                        {
                            Console.WriteLine("***** "+p+" *****");
                            Console.WriteLine(previousTime[p]);
                            Console.WriteLine(runner.plan.GetLocations().ElementAt<List<Move>>(0)[p]);
                            Console.WriteLine(locationsAtTime[p]);
                        }*/
                        cost += executePlan(runner, instance, newPaths, lazyOrEager, chance);
                       /* if (checkIfColide(previousTime))
                        {
                            Console.WriteLine("***** PROBLEM6 *****");
                            Console.ReadLine();
                        }*/
                        return cost;
                    }
                }
                else if(lazyOrEager == LazyOrEager.Eager)
                {
                    if (i!=0 && delayMoves.Count > 0)
                    {
                        Console.WriteLine("***** REPLAN *****");
                        replanCounter++;
                        /*if (checkIfColide(previousTime))
                        {
                            if(previousTime == locationsAtTime)
                                Console.WriteLine("asd");
                            Console.WriteLine("immposible!!");
                        }*/
                        if (previousTime == null)
                        {
                            Console.WriteLine("i=" + i);
                            Console.Read();
                        }
                        Stopwatch replanStopwath = new Stopwatch();
                        replanStopwath.Start();
                        replan(previousTime, instance, runner);
                        replanStopwath.Stop();
                        replanTime += replanStopwath.Elapsed.TotalMilliseconds;
                        cost += executePlan(runner, instance, newPaths, lazyOrEager, chance);
                        return cost;
                    }
                }
                delayAgents = new List<int>();
                delayMoves  = new List<Move>();
                newPaths.AddLast(locationsAtTime);
                previousTime    = locationsAtTime;
               /* if (checkIfColide(previousTime))
                {
                    Console.WriteLine("***** PROBLEM2 *****");
                    Console.ReadLine();
                }*/
                if (node != lLocations.Last)
                {
                    node            = node.Next;
                    locationsAtTime = node.Value;
                }
                if (dynamicSize != lLocations.Count)
                    dynamicSize = lLocations.Count;
              /*  if (checkIfColide(previousTime))
                {
                    Console.WriteLine("***** PROBLEM3 *****");
                    Console.ReadLine();
                }*/
            }
            return cost;
        }


        public void replan(List<Move> previousTime, ProblemInstance instance, Run runner)
        {
            AgentState[] aStart = instance.m_vAgents;
            for (int i = 0; i < previousTime.Count; i++)
            {
                aStart[i] = new AgentState(previousTime[i].x, previousTime[i].y, aStart[i].agent);
            }
            instance.Init(aStart, instance.m_vGrid);
            runner.SolveGivenProblem(instance);
        }

        public void agentDelay(List<int> delayAgents,List<Move> delayMoves, Run runner, List<Move> pre, int currentTime)
        {
            Dictionary<int, Move> agentToMove       = new Dictionary<int, Move>();
            Dictionary<int, Move> previousToMove    = new Dictionary<int, Move>();
            LinkedList<List<Move>> locationsAtTimes = runner.plan.GetLocations();
            List<Move> newTime  = null;
            bool first          = true;
            int i;
            List<Move> locations;
            LinkedListNode<List<Move>> node = locationsAtTimes.First;
            locations                       = node.Value;
            while (pre != null && locations != pre)
            {
                node        = node.Next;
                locations   = node.Value;
            }
            if (pre != null)
                node = node.Next;
            for (i = currentTime; i < locationsAtTimes.Count; i++)
            {
                locations = node.Value;
               /* if (pre != null && checkIfColide(pre))
                {
                    Console.WriteLine("***** PROBLEM9 *****");
                    Console.ReadLine();
                }*/
                if (!first)
                {
                    foreach (int agentIndex in delayAgents)
                    {
                      /*  if (pre!=null && checkIfColide(pre))
                        {
                            Console.WriteLine("***** PROBLEM10 *****");
                            Console.ReadLine();
                        }*/
                        agentToMove[agentIndex] = new TimedMove((TimedMove)locations.ElementAt<Move>(agentIndex));
                       /* if (pre != null && checkIfColide(pre))
                        {
                            Console.WriteLine("***** PROBLEM14 *****");
                            Console.ReadLine();
                        }*/
                        locations[agentIndex] = new TimedMove((TimedMove)previousToMove[agentIndex]);
                        /*if (pre != null && checkIfColide(pre))
                        {
                            Console.WriteLine("***** PROBLEM15 *****");
                            Console.ReadLine();
                        }*/
                        previousToMove[agentIndex] = new TimedMove((TimedMove)agentToMove[agentIndex]);
                       /* if (pre != null && checkIfColide(pre))
                        {
                            Console.WriteLine("***** PROBLEM16 *****");
                            Console.ReadLine();
                        }*/
                        if (i == locationsAtTimes.Count && (locations[agentIndex].x != agentToMove[agentIndex].x || locations[agentIndex].y != agentToMove[agentIndex].y))
                        {
                            newTime = new List<Move>();
                            foreach (Move mv in locations)
                            {
                                newTime.Add(new TimedMove((TimedMove)mv));
                            }
                            newTime[agentIndex] = new TimedMove((TimedMove)agentToMove[agentIndex]);
                            foreach (Move mv in newTime)
                            {
                                ((TimedMove)mv).time = i;
                            }
                        }
                     /*   if (pre != null && checkIfColide(pre))
                        {
                            Console.WriteLine("***** PROBLEM11 *****");
                            Console.ReadLine();
                        }*/
                    }
                    foreach (Move mv in locations)
                    {
                        ((TimedMove)mv).time = i - 1;
                    }
                   /* if (pre != null && checkIfColide(pre))
                    {
                        Console.WriteLine("***** PROBLEM12 *****");
                        Console.ReadLine();
                    }*/
                }
                else
                {
                    foreach (int agentIndex in delayAgents)
                    {
                        previousToMove[agentIndex] = new TimedMove((TimedMove)locations.ElementAt<Move>(agentIndex));
                    }
                }
               /* if (pre != null && checkIfColide(pre))
                {
                    Console.WriteLine("***** PROBLEM13 *****");
                    Console.ReadLine();
                }*/
                first = false;
                node = node.Next;
            }
            if (newTime != null)
            {
                locationsAtTimes.AddLast(newTime);
            }
        }

        public bool checkIfColide(List<Move> locationsAtTime)
        {
            Move aMove, bMove;
            for (int i = 0; i < locationsAtTime.Count; i++)
                for (int j = i + 1; j < locationsAtTime.Count; j++)
                {
                    aMove = locationsAtTime[i];
                    bMove = locationsAtTime[j];
                    if (aMove != bMove && ((Move)aMove).IsColliding(bMove))
                    {
                        Console.WriteLine("***** COLLIDE AT (" + aMove.x + "," + aMove.y + ") *****");
                        return true;
                    }

                }
            return false;
            /*
            foreach(Move aMove in locationsAtTime)
                foreach(Move bMove in locationsAtTime)
                {
                    if(aMove != bMove && aMove.IsColliding(bMove))
                    {
                        Console.WriteLine("***** COLLIDE AT ("+ aMove.x +","+ aMove.y + ") *****");
                        return true;
                    }
                }
            return false;*/
        }
        /// <summary>
        /// This is the starting point of the program. 
        /// </summary>
        static void Main(string[] args)
        {
            Program me = new Program();
            Program.RESULTS_FILE_NAME = Process.GetCurrentProcess().ProcessName + ".csv";
            TextWriterTraceListener tr1 = new TextWriterTraceListener(System.Console.Out);
            Debug.Listeners.Add(tr1);
            //if (System.Diagnostics.Debugger.IsAttached)
            //    Constants.MAX_TIME = int.MaxValue;

            if (Directory.Exists(Directory.GetCurrentDirectory() + "\\Instances") == false)
            {
                Directory.CreateDirectory(Directory.GetCurrentDirectory() + "\\Instances");
            }

            Program.onlyReadInstances = false;

            int instances = 1;
            //int instances = 50;

            //bool runGrids = false;
            //bool runDragonAge = true;
            bool runDragonAge = false;
            bool runGrids = true;
            bool runMazesWidth1 = false;
            bool runSpecific = false;

            if (runGrids == true)
            {
                int[] gridSizes = new int[] { 8, };
                //int[] agentListSizes = new int[] { 2, 3, 4 };

                //int[] gridSizes = new int[] { 6, };
                int[] agentListSizes = new int[] { 10 };
                //int[] agentListSizes = new int[] { /*2,*/ 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
                // Note that success rate drops almost to zero for EPEA* and A*+OD/SIC on 40 agents.
            
                //int[] gridSizes = new int[] { 32, };
                //int[] agentListSizes = new int[] { /*5, 10, 15, 20, 25, 30, 35,*/ 40, /*45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150*/ };
                //int[] agentListSizes = new int[] { 10, 20, 30, 40, 50, 60 };

                //int[] obstaclesPercents = new int[] { 20, };
                //int[] obstaclesPercents = new int[] { /*0, 5, 10, 15, 20, 25, 30, 35, */20, 30, 40};
                int[] obstaclesPercents = new int[] { 0, /*5, 10,*/ /*15,*/ /*20, 25, 30, 35, 20, 30, 40*/ };
                me.RunExperimentSet(gridSizes, agentListSizes, obstaclesPercents, instances);
            }
            else if (runDragonAge == true)
                me.RunDragonAgeExperimentSet(instances, Program.daoMapFilenames); // Obstacle percents and grid sizes built-in to the maps.
            else if (runMazesWidth1 == true)
                me.RunDragonAgeExperimentSet(instances, Program.mazeMapFilenames); // Obstacle percents and grid sizes built-in to the maps.
            else if (runSpecific == true)
            {
                me.RunInstance("Instance-5-15-6-13");
                //me.RunInstance("Instance-5-15-3-792");
                //me.RunInstance("Instance-5-15-3-792-4rows");
                //me.RunInstance("Instance-5-15-3-792-3rows");
                //me.RunInstance("Instance-5-15-3-792-2rows");
                //me.RunInstance("corridor1");
                //me.RunInstance("corridor2");
                //me.RunInstance("corridor3");
                //me.RunInstance("corridor4");
            }

            // A function to be used by Eric's PDB code
            //me.runForPdb();
            Console.WriteLine("*********************THE END**************************");
            Console.ReadLine();
        }    
    }
}
