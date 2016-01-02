/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	/* Use a Adjacency List to hold the graph */
	private Map<Vertex, ArrayList<Vertex>> adjLst;
	private Map<Edge, Double> edgeLen;
	private Map<Vertex, Double> totLen;
	private int numVertices, numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjLst = new HashMap<Vertex, ArrayList<Vertex>>();
		edgeLen = new HashMap<Edge, Double>();
		totLen = new HashMap<Vertex, Double>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		/* Return a copy of the set of vertices. We this in order to
		 * not expose the internals of our graph to the outside world */
		return new HashSet<GeographicPoint>(adjLst.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	/** 
	 * Helper method to check if a given vertex exists in the graph 
	 */
	private boolean hasVertex(GeographicPoint location) {
		return adjLst.keySet().contains(location);
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		/* Check if the vertex was already in 
		 * the graph, or the parameter is null */
		if ((location == null) || hasVertex(location))
			return false;
		
		/* Add to graph and increment number of vertices */
		adjLst.put(new Vertex(location.x, location.y, Double.MAX_VALUE), 
				new ArrayList<Vertex>());
		totLen.put(new Vertex(location.x, location.y, Double.MAX_VALUE), Double.MAX_VALUE);
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		/* Check for:
		 * 1. The points have not already been added as nodes to the graph
		 * 2. Any of the arguments is null 
		 * 3. If the length is less than 0  */
		if ((from == null) || (to == null) || 
				(!hasVertex(to)) || (!hasVertex(from)) || 
				(roadName == null) || (roadType == null) || (length < 0))
			throw new IllegalArgumentException();
		
		/* Create a new Edge object */
		Edge e = new Edge(from, to);
		edgeLen.put(e, length);
		
		/* Add to graph and increment number of edges */
		adjLst.get(from).add(new Vertex(to.x, to.y, Double.MAX_VALUE));
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		/* Check sanity of arguments */
		if ((start == null) || (goal == null) || (nodeSearched == null) || 
				(!hasVertex(start)) || (!hasVertex(goal)))
			return null;
		
		/* Generate the parent Map */
		Map<Vertex, Vertex> parentMap = 
				bfsCore(start, goal, nodeSearched);
		
		/* Generate the route based on the parent Map */
		return generateRoute(parentMap, start, goal);
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		/* Check sanity of arguments */
		if ((start == null) || (goal == null) || (nodeSearched == null) || 
				(!hasVertex(start)) || (!hasVertex(goal)))
			return null;
		
		/* Generate the parent Map */
		Map<Vertex, Vertex> parentMap = 
				getParentMap(start, goal, nodeSearched, true);
		
		/* Generate the route based on the parent Map */
			return generateRoute(parentMap, start, goal);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		/* Check sanity of arguments */
		if ((start == null) || (goal == null) || (nodeSearched == null) || 
				(!hasVertex(start)) || (!hasVertex(goal)))
			return null;
		
		/* Generate the parent Map */
		Map<Vertex, Vertex> parentMap = 
				getParentMap(start, goal, nodeSearched, false);
		
		/* Generate the route based on the parent Map */
			return generateRoute(parentMap, start, goal);
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("This is a Graph with " + 
	getNumVertices() + " vertices and " +  getNumEdges() + " edges.");
		sb.append("\n");
		Set<Vertex> nodeSet = adjLst.keySet();
		for (GeographicPoint g : nodeSet) {
			sb.append(g);
			sb.append("->");
			sb.append("{");
			ArrayList<Vertex> lst = adjLst.get(g);
			for (Vertex j : lst)
				sb.append(j + "| ");
			sb.deleteCharAt(sb.length() - 1);
			sb.deleteCharAt(sb.length() - 1);
			sb.append("}\n");
		}
		return sb.toString();
	}
	
	/** 
	 * Helper function to get the neighbors for a given GeographicPoint
	 */
	private List<Vertex> getNeighbors(Vertex v) {
		return new ArrayList<Vertex>(adjLst.get(v));
	}
	
	/** 
	 * Helper function to do the core work of the Dijkstra algorithm
	 * and generate the parent map.
	 */ 
	private Map<Vertex, Vertex> getParentMap(GeographicPoint start,
		     GeographicPoint goal, 
		     Consumer<GeographicPoint> nodeSearched, 
		     boolean isDijkstra) {
		
		/* Initialize the parent map */
		Map<Vertex, Vertex> parentMap = new HashMap<Vertex, Vertex>();
		PriorityQueue<Vertex> pQue;
		
		/* Based on whether we are using dijkstra or astar, initialize the 
		 * PriorityQUeue. For Dijkstra, the comparable interface's implemented 
		 * compareTo() method will be used (from vertex class). Whereas for 
		 * astar, we provide a comparator object while initializing the 
		 * PriorityQueue.*/
		if (isDijkstra)
			pQue = new PriorityQueue<Vertex>();
		else
			/* For astar, the comparison function not only takes into account
			 * the distance from start, but also the distance (direct) to the
			 * goal. */
			pQue = new PriorityQueue<Vertex>(new Comparator<Vertex>() {
				public int compare(Vertex v1, Vertex v2) {
					double v1Dist = v1.getTotalDist() + v1.distance(goal);
					double v2Dist = v2.getTotalDist() + v2.distance(goal);
					if (v1Dist > v2Dist) {
						return 1;
					}
					else if (v1Dist < v2Dist) {
						return -1;
					}
					else
						return 0;
				}
			});
			
		/* Initialize all elements in the priority queue to be Infinite priority */
		for (Vertex v : adjLst.keySet()) {
			pQue.add(v);
		}
		
		/* Initialize the visited set */
		Set<Vertex> visited = new HashSet<Vertex>();
		
		/* Add the Start vertex to the PriorityQueue */
		pQue.add(new Vertex(start.x, start.y, 0));
		
		while (!pQue.isEmpty()) {
			/* deque into curr */
			Vertex cur = pQue.remove();
			nodeSearched.accept(cur);
			/* check if cur has been visited */
			if (!visited.contains(cur)) {
				/* Add cur to visited */
				visited.add(cur);
				/* If cur is goal we are done */
				if (cur.equals(goal))
					break;
				/* For every neighbor of cur not in visited set */
				List<Vertex> neighbors = getNeighbors(cur);
				for (Vertex n : neighbors) {
					if (!visited.contains(n)) {
						Edge e = new Edge(cur, n);
						double totalDist = cur.getTotalDist() + edgeLen.get(e);
						/* Check if distance to n through cur is 
						 * lesser than through any earlier paths */
						if (totalDist < totLen.get(n)) {
							/* Update the distance to n in the totLen map */
							totLen.replace(n, totLen.get(n), totalDist);
							/* Set the total distance of the vertex as well */
							n.setTotalDist(totalDist);
							/* Add the vertex to the priority queue */
							pQue.add(n);
							/* If the parent map already contains a
							 * mapping for n, remove that mapping */
							if (parentMap.containsKey(n))
								parentMap.remove(n);
							/* Add the new mapping */
							parentMap.put(n, cur);
						}
					}
				}
			}
		}
		return parentMap;
	}
	
	/** 
	 * Helper function to do the core work of the BFS algorithm
	 * and generate the parent map.
	 */ 
	private Map<Vertex, Vertex> bfsCore(GeographicPoint start,
		     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		
		/* Initialize all the data structures needed for BFS */
		Queue<Vertex> q = new LinkedList<Vertex>();
		Set<Vertex> visited = new HashSet<Vertex>();
		Map<Vertex, Vertex> parentMap = 
				new HashMap<Vertex, Vertex>();

		/* Add the starting node to the queue and mark it visited */
		q.add(new Vertex (start.x, start.y, Double.MAX_VALUE));
		visited.add(new Vertex (start.x, start.y, Double.MAX_VALUE));	
		
		/* Keep running till the queue becomes empty */
		while (!q.isEmpty()) {
			Vertex cur = q.remove();
			/* Report searched node to the consumer */
			nodeSearched.accept(cur);
			if (cur.equals(goal)) {
				break;
			}
			/* Add every non-visited neighbor to the visited
			 * list, the queue and the parent map */
			List<Vertex> neighbors = getNeighbors(cur);
			for (Vertex n : neighbors) {
				if (!visited.contains(n)) {
					visited.add(n);
					q.add(n);
					parentMap.put(n, cur);
				}
			}
		}	
		return parentMap;
	}
	
	/** 
	 * Helper function to generate the route based on the parent map
	 */ 
	private List<GeographicPoint> generateRoute(
			Map<Vertex, Vertex> parentMap, 
			GeographicPoint start, GeographicPoint goal) {
		
		List<GeographicPoint> route = new ArrayList<GeographicPoint>();
		
		/* First Add the goal to the path */
		route.add(0, goal);
		GeographicPoint node = goal;
		/* Then add the remaining vertices to the
		 * path by looking up in the parent map */
		while (!node.equals(start)) {
			node = parentMap.get(node);
			/* No path exists from start to goal */
			if (node == null)
				return null;
			route.add(0, node);
		}
		return route;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.createIntersectionsFile("data/testdata/simpletest2.map", 
                "data/intersections/simpletest2.intersections");
		GraphLoader.loadRoadMap("data/testdata/simpletest2.map", theMap);
		System.out.println("DONE.");
		//System.out.println(theMap);
		List<GeographicPoint> path =  theMap.dijkstra(new GeographicPoint(4, 0), new GeographicPoint(8, -1));
		
		System.out.println();
		System.out.println();
		System.out.println();
		for (GeographicPoint p : path) {
			System.out.println(p);
		}
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.868629, -117.215393);
		GeographicPoint end = new GeographicPoint(32.868629, -117.215393);
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
