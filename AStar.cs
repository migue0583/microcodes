using System;
using System.Drawing;
using System.Collections.Generic;

public class AStar
{
    /// <summary>
    ///     A* finds a path from start to goal by using a grid matrix.
    ///     You can find an example of use at the bottom of this class.
    /// </summary>
    /// <param name="start">Indicates the starting node.</param>
    /// <param name="goal">Indicates the goal or target node</param>
    /// <param name="gridMatrix">
    ///     A matrix representing a grid or terran with a value of 0 (zero)
    ///     if the node is not valid (blocked) or distinct to 0, if the 
    ///     node is valid (traversable). This is not an adjacency matrix, 
    ///     strictly speaking.
    /// </param>
    /// <returns>
    ///     A list with the nodes that form the path from start to goal.
    ///     If the path is returned null, there is no way to reach the goal 
    ///     from the start.
    /// </returns>
    public static List<Point> FindPath(Point start, Point goal, int[,] gridMatrix)
    {
        //The quantity of rows and columns that the grid has
        int rowsQuantity = gridMatrix.GetLength(0);
        int colsQuantity = gridMatrix.GetLength(1);

        //The set of discovered nodes that may need to be (re-)expanded.
        //Initially, only the start node is known.
        //This is usually implemented as a min-heap or priority queue 
        //rather than a hash-set or a list.
        List<Point> openSet = new List<Point>();
        openSet.Add(start);

        //For node n, cameFrom[n] is the node immediately 
        //preceding it on the cheapest path from start
        //to n currently known.
        Dictionary<Point, Point> cameFrom = new Dictionary<Point,Point>();

        //For node n, gScore[n] is the cost of the cheapest 
        //path from start to n currently known.
        Dictionary<Point, double> gScore = new Dictionary<Point, double>();
        //Map with default value of Infinity
        FillMapWithInfinity(gScore, rowsQuantity, colsQuantity);
        gScore[start] = 0;

        //For node n, fScore[n] := gScore[n] + h(n). 
        //fScore[n] represents our current best guess as to
        //how short a path from start to finish can be if it goes through n.
        Dictionary<Point, double> fScore = new Dictionary<Point, double>();
        //Map with default value of Infinity
        FillMapWithInfinity(fScore, rowsQuantity, colsQuantity);
        fScore[start] = h(start, goal);

        while (openSet.Count != 0)
        {
            //This operation can occur in O(1) time if openSet is a 
            //min-heap or a priority queue
            //current := the node in openSet having the lowest fScore[] value
            Point current = GetMinFromOpenSet(openSet, fScore);

            if(current == goal)
            {//The goal is reached. Successful end of the algorithm.
                return ReconstructPath(cameFrom, current);
            }
            openSet.Remove(current);

            List<Point> neighborhood = GetNeighborhood(
                current, gridMatrix);

            foreach (Point neighbor in neighborhood)
            { 
                // d(current,neighbor) is the weight of the edge from current to neighbor
                // tentative_gScore is the distance from start to the neighbor through current
                double tentativeGscore = gScore[current] + d(current, neighbor);

                if (tentativeGscore < gScore[neighbor])
                {
                    // This path to neighbor is better than any previous one. Record it!
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGscore;
                    fScore[neighbor] = gScore[neighbor] + h(neighbor, goal);
                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }
        // Open set is empty but goal was never reached
        //Unsuccessful end of the algorithm.
        return null;
    }

    //The d method stimates the distance 
    //to reach from node A to B, by calculating the euclidian
    //distance between the two nodes.
    private static double d(Point nodeA, Point nodeB) {
        return Math.Sqrt(Math.Pow(nodeB.X - nodeA.X, 2) 
                            + Math.Pow(nodeB.Y - nodeA.Y, 2));
    }

    //This is the infamous Heuristic method/function.
    //In this case, it will stimate the effort 
    //required to reach from a node to the goal 
    //as the distances between them.
    private static double h(Point node, Point goal)
    {
        return d(node, goal);
    }

    //Creates a list tracing back the nodes 
    //traversed to reach the goal. This represents
    //the path found.
    private static List<Point> ReconstructPath(
        Dictionary<Point, Point> cameFrom, Point current)
    {
        List<Point> totalPath = new List<Point>();
        totalPath.Add(current);

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            totalPath.Insert(0, current);
        }

        return totalPath;
    }

    //Creates a list with all the nodes ayacent 
    //to a particular node. It uses the grid matrix
    //to know if it a potential neighbor is valid or not. 
    //If the matrix has a value of 0, the node is not 
    //valid, and it is not added as a neighbor. 
    //Otherwise, it is added.
    private static List<Point> GetNeighborhood(
        Point node, int[,] adyMatrix)
    {
        List<Point> neighborhood = new List<Point>();

        for (int r = -1; r <= 1; r++)
        {
            for (int c = -1; c <= 1; c++)
            {
                Point neighbor = new Point(node.X + c, node.Y + r);
                if (neighbor == node)
                {
                    continue;
                }
                if (IsValidNode(neighbor, adyMatrix))
                {
                    neighborhood.Add(neighbor);
                }
            }
        }
        return neighborhood;
    }

    private static bool IsValidNode(
        Point node, int[,] gridMatrix)
    {
        int cantFilas = gridMatrix.GetLength(0);
        int cantCols = gridMatrix.GetLength(1);

        if (node.X < 0 || node.Y < 0)
        {//Outside the matrix. 
            return false;
        }

        if (node.X >= cantCols || node.Y >= cantFilas)
        {//Outside the matrix. 
            return false;
        }
        //Returns true if the grid matrix has a value distinct than
        //zero in the reviewed node.
        return gridMatrix[node.Y, node.X] != 0;
    }

    //Traverses the openSet list looking for the
    //node with the lowest fScore value.
    //This method would be unnecessary if the 
    //openSet where a priority queue.
    private static Point GetMinFromOpenSet(
        List<Point> openSet, Dictionary<Point, double> fScore)
    {
        double minScore = int.MaxValue;
        Point minNode = new Point(-1, -1);

        //Traverse each nod in the open set.
        foreach (Point node in openSet)
        {
            double score = fScore[node];
            if (score <= minScore)
            {
                minScore = score;
                minNode = node;
            }
        }
        return minNode;
    }

    //Prefills each node of the grid matrix with infinity as value.
    private static void FillMapWithInfinity(
        Dictionary<Point, double> map, int rowsQty, int colsQty)
    {
        for (int r = 0; r < rowsQty; r++)
        {
            for (int c = 0; c < colsQty; c++)
            {
                Point node = new Point(c, r);
                map[node] = int.MaxValue;
            }
        }
    }

    static void Main()
    {
        int[,] gridMatrix = new int[,]{
            {1, 1, 1, 0},
            {1, 0, 1, 0},
            {1, 1, 0, 1},
            {0, 0, 0, 1},
            {0, 0, 1, 1},
        };
         
        //Starting node at row 0, column 0
        Point start = new Point(0, 0);
        //Goal node at row 4, column 2
        Point goal = new Point(2, 4);

        List<Point> path = AStar.FindPath(start, goal, gridMatrix);
        if (path != null)
        {
            Console.WriteLine("This is the path from " + start + " to " + goal);  
            foreach (Point p in path)
            {
                Console.WriteLine(p);
            }
        }
        else 
        {
            Console.WriteLine("There is no path from " + start + " to " + goal);  
        }
            
        Console.Read();
    }
}

