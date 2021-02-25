import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AStar {

	/**
	 * A* finds a path from start to goal by using a grid matrix. You can find an
	 * example of use at the bottom of this class.
	 * 
	 * @param start      Indicates the starting node.
	 * @param goal       Indicates the goal or target node
	 * @param gridMatrix A matrix representing a grid or terran with a value of 0
	 *                   (zero) if the node is not valid (blocked) or distinct to 0,
	 *                   if the node is valid (traversable). This is not an
	 *                   adjacency matrix, strictly speaking.
	 * @return A list with the nodes that form the path from start to goal. If the
	 *         path is returned null, there is no way to reach the goal from the
	 *         start.
	 */
	public static List<Node> findPath(Node start, Node goal, int[][] gridMatrix) {
		// The quantity of rows and columns that the grid has
		int rowsQuantity = gridMatrix.length;
		int colsQuantity = gridMatrix[0].length;

		// The set of discovered nodes that may need to be (re-)expanded.
		// Initially, only the start node is known.
		// This is usually implemented as a min-heap or priority queue
		// rather than a hash-set or a list.
		List<Node> openSet = new ArrayList<Node>();
		openSet.add(start);

		// For node n, cameFrom[n] is the node immediately
		// preceding it on the cheapest path from start
		// to n currently known.
		Map<Node, Node> cameFrom = new HashMap<Node, Node>();

		// For node n, gScore[n] is the cost of the cheapest
		// path from start to n currently known.
		Map<Node, Double> gScore = new HashMap<Node, Double>();
		// Map with default value of Infinity
		fillMapWithInfinity(gScore, rowsQuantity, colsQuantity);
		gScore.put(start, 0d);

		// For node n, fScore[n] := gScore[n] + h(n).
		// fScore[n] represents our current best guess as to
		// how short a path from start to finish can be if it goes through n.
		Map<Node, Double> fScore = new HashMap<Node, Double>();
		// Map with default value of Infinity
		fillMapWithInfinity(fScore, rowsQuantity, colsQuantity);
		fScore.put(start, h(start, goal));

		while (!openSet.isEmpty()) {
			// This operation can occur in O(1) time if openSet is a
			// min-heap or a priority queue
			// current := the node in openSet having the lowest fScore[] value
			Node current = getMinFromOpenSet(openSet, fScore);

			if (current.equals(goal)) {// The goal is reached. Successful end of the algorithm.
				return reconstructPath(cameFrom, current);
			}
			openSet.remove(current);

			List<Node> neighborhood = getNeighborhood(current, gridMatrix, rowsQuantity, colsQuantity);

			for (Node neighbor : neighborhood) {
				// d(current,neighbor) is the weight of the edge from current to neighbor
				// tentative_gScore is the distance from start to the neighbor through current
				Double tentativeGscore = gScore.get(current) + d(current, neighbor);

				if (tentativeGscore < gScore.get(neighbor)) {
					// This path to neighbor is better than any previous one. Record it!
					cameFrom.put(neighbor, current);
					gScore.put(neighbor, tentativeGscore);
					fScore.put(neighbor, gScore.get(neighbor) + h(neighbor, goal));
					if (!openSet.contains(neighbor)) {
						openSet.add(neighbor);
					}
				}
			}
		}
		// Open set is empty but goal was never reached
		// Unsuccessful end of the algorithm.
		return null;
	}

	// The d method stimates the distance
	// to reach from node A to B, by calculating the euclidian
	// distance between the two nodes.
	private static double d(Node nodeA, Node nodeB) {
		return Math.sqrt(Math.pow(nodeB.col - nodeA.col, 2) + Math.pow(nodeB.row - nodeA.row, 2));
	}

	// This is the infamous Heuristic method/function.
	// In this case, it will stimate the effort
	// required to reach from a node to the goal
	// as the distances between them.
	private static double h(Node node, Node goal) {
		return d(node, goal);
	}

	// Creates a list tracing back the nodes
	// traversed to reach the goal. This represents
	// the path found.
	private static List<Node> reconstructPath(Map<Node, Node> cameFrom, Node current) {
		List<Node> totalPath = new ArrayList<Node>();
		totalPath.add(current);

		while (cameFrom.containsKey(current)) {
			current = cameFrom.get(current);
			totalPath.add(0, current);
		}

		return totalPath;
	}

	// Creates a list with all the nodes ayacent
	// to a particular node. It uses the grid matrix
	// to know if it a potential neighbor is valid or not.
	// If the matrix has a value of 0, the node is not
	// valid, and it is not added as a neighbor.
	// Otherwise, it is added.
	private static List<Node> getNeighborhood(Node node, int[][] adyMatrix, int rowsQty, int colsQty) {
		List<Node> neighborhood = new ArrayList<Node>();

		for (int r = -1; r <= 1; r++) {
			for (int c = -1; c <= 1; c++) {
				Node neighbor = new Node(node.row + r, node.col + c);
				if (neighbor.equals(node)) {
					continue;
				}
				if (isValidNode(neighbor, adyMatrix, rowsQty, colsQty)) {
					neighborhood.add(neighbor);
				}
			}
		}
		return neighborhood;
	}

	private static boolean isValidNode(Node node, int[][] gridMatrix, int rowsQty, int colsQty) {

		if (node.col < 0 || node.row < 0) {// Outside the matrix.
			return false;
		}

		if (node.col >= colsQty || node.row >= rowsQty) {// Outside the matrix.
			return false;
		}
		// Returns true if the grid matrix has a value distinct than
		// zero in the reviewed node.
		return gridMatrix[node.row][node.col] != 0;
	}

	// Traverses the openSet list looking for the
	// node with the lowest fScore value.
	// This method would be unnecessary if the
	// openSet where a priority queue.
	private static Node getMinFromOpenSet(List<Node> openSet, Map<Node, Double> fScore) {
		double minScore = Double.MAX_VALUE;
		Node minNode = null;

		// Traverse each nod in the open set.
		for (Node node : openSet) {
			Double score = fScore.get(node);
			if (score <= minScore) {
				minScore = score;
				minNode = node;
			}
		}
		return minNode;
	}

	// Prefills each node of the grid matrix with infinity as value.
	private static void fillMapWithInfinity(Map<Node, Double> map, int rowsQty, int colsQty) {
		for (int r = 0; r < rowsQty; r++) {
			for (int c = 0; c < colsQty; c++) {
				Node node = new Node(r, c);
				map.put(node, Double.MAX_VALUE);
			}
		}
	}

	static class Node {
		public int col;
		public int row;

		public Node(int row, int col) {
			this.col = col;
			this.row = row;
		}

		public boolean equals(Object obj) {
			if (obj instanceof Node) {
				Node other = (Node) obj;
				return this.row == other.row && this.col == other.col;
			}
			return false;
		}

		public String toString() {
			return "[Row=" + row + ", Col=" + col + "]";
		}

		public int hashCode() {
			return toString().hashCode();
		}
	}

	public static void main(String[] args) {

		int[][] gridMatrix = new int[][] { 
			{ 1, 1, 1, 0 }, 
			{ 1, 0, 1, 0 }, 
			{ 1, 1, 0, 1 }, 
			{ 0, 0, 0, 1 },
			{ 0, 0, 1, 1 } 
		};

		// Starting node at row 0, column 0
		AStar.Node start = new AStar.Node(0, 0);
		// Goal node at row 4, column 2
		AStar.Node goal = new AStar.Node(4, 2);

		List<AStar.Node> path = AStar.findPath(start, goal, gridMatrix);
		if (path != null) {
			System.out.println("This is the path from " + start + " to " + goal);
			for (AStar.Node p : path) {
				System.out.println(p);
			}
		} else {
			System.out.println("There is no path from " + start + " to " + goal);
		}
	}
}
