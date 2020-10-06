/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


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
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> map;
	private Set<GeographicPoint> vertices;
	private int numVertices;
	private int numEdges;
	private int dijkstraNodes;
	private int aStarNodes;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		map = new HashMap<GeographicPoint, MapNode>();
		vertices = new HashSet<GeographicPoint>();
		numVertices = 0;
		numEdges = 0;
		dijkstraNodes = 0;
		aStarNodes = 0;
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
		return this.vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return this.numEdges;
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
		
		if(location == null) 		  return false;		// Return false if location object is null.
		if(map.containsKey(location)) return false;		// Return false if that location is already exist.
	
		
		
		
		MapNode vertex = new MapNode(location);			// Create a node with given location.
		map.put(location, vertex);						// Add the node and its location to the map.
		numVertices++;									// Add one to the vertices count.
		vertices.add(location);							// Add this location to Vertices hashSet
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

		//Throw an IllegalArgumentException if either of the two points are not in the graph already, 
		//if any of the arguments is null, or if length is less than 0.	
		if(!map.containsKey(from) || !map.containsKey(to) || roadName == null || roadType == null || length<0) {
			throw new IllegalArgumentException();
		}
		MapEdge currEdge = new MapEdge(from,to,roadName, roadType, length);   // Create an edge with given parameters.
		map.get(from).addEdge(currEdge);									  // Add this edge to the map and its location should be same with the key.
		numEdges++;															  // Add one to the edges count.
		
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
		// TODO: Implement this method in WEEK 3
		
		if(start == null || goal == null) {
			return null;
		}
		boolean found = false;
		/** Initialize visited set, queue, parentMap **/
		Set<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> q = new LinkedList<MapNode>();
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		
		/**   Breadth-First Search   **/
		q.add(startNode);
		while(!q.isEmpty()) {
			MapNode curr = q.remove();
			nodeSearched.accept(curr.getLocation());     // For the visualization button in App.
			if(curr==goalNode) {
				found = true;
				break;
			}
			for(MapEdge neighbor : curr.getEdgeList()) {
				MapNode i = map.get(neighbor.getEndPoint());
					if(!visited.contains(i)) {
						q.add(i);
						visited.add(i);
						parentMap.put(i, curr);
				}
			}	
		}
		if(!found) return null;     // If goal node is not found, return null.
		
		List<GeographicPoint> path = constructPath(parentMap, startNode, goalNode);
		return path;
	
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
		if(dijkstraNodes != 0) dijkstraNodes = 0; // For testing
		PriorityQueue<MapNode> PQ = new PriorityQueue<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		boolean found = false;

		//Initialize all the distances to infinity.
		initializeNodeDistancesToInfinity(start);
		
		/** Dijkstra Algorithm using PriorityQueue **/
		PQ.add(startNode);
		while(!PQ.isEmpty()) {
			MapNode curr = PQ.remove();
			nodeSearched.accept(curr.getLocation());
			dijkstraNodes++; // Show visited nodes using show visualization button in the app.
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr == goalNode) {
					found = true;
					break;
				}
				for(MapEdge neighbor : curr.getEdgeList()) {
					MapNode i = map.get(neighbor.getEndPoint());
					if(!visited.contains(i)) {
						if(i.getDistance() > curr.distance + neighbor.getLength()) {
							i.setDistance(curr.distance + neighbor.getLength());
							parentMap.put(i, curr);
							PQ.add(i);
						}
					}
				}
			}
		}
		
		if(!found) return null;     // If goal node is not found, return null.
		
		List<GeographicPoint> path = constructPath(parentMap, startNode, goalNode);
		return path;

	}
	
	private void initializeNodeDistancesToInfinity(GeographicPoint start) {
		// Loop through every node in the map.
		for(MapNode node : map.values()) {
			// If it is starter node, set distance to zero. Otherwise set distance to infinity.
			if(node.getLocation().equals(start)) {
				node.setDistance(0);
			}
			else {
				node.setDistance(Double.POSITIVE_INFINITY);
			}
		}
	}
	
	private List<GeographicPoint> constructPath(Map<MapNode,MapNode> parentMap,
												MapNode startNode, MapNode goalNode) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode iter = goalNode;                // Start iterate by goal node.
		while(iter != startNode) {				// When iteration node finds the starting node, stop counting.
			path.addFirst(iter.getLocation());  // Add last discovered location to the list.
			iter = parentMap.get(iter);		    // Set iteration node to its parent node. (It means who discovered the last node).
		}
		path.addFirst(startNode.getLocation());	// Lastly, add last node to the path.
		return path;
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
	
	public static Comparator<MapNode> aComparator = new Comparator<MapNode>() {
		@Override
		public int compare(MapNode m1, MapNode m2){
			if (m1.getDistance()+m1.getDistanceToGoal() < m2.getDistance() + m2.getDistanceToGoal()) {
				return -1;
			}
			if (m1.getDistance()+m1.getDistanceToGoal() > m2.getDistance() + m2.getDistanceToGoal()) {
				return 1;
			}
			return 0;
		}
	};
	

	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if(aStarNodes != 0) aStarNodes = 0; // For testing
		PriorityQueue<MapNode> PQ = new PriorityQueue<MapNode>(aComparator);
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		boolean found = false;
		
		initializeNodeDistancesToInfinity(start);
		startNode.setDistanceToGoal(start.distance(goal));
		
		PQ.add(startNode);
		while(!PQ.isEmpty()) {
			MapNode curr = PQ.remove();
			nodeSearched.accept(curr.getLocation());
			aStarNodes++;// Show visited nodes using show visualization button in the app.
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr == goalNode) {
					found = true;
					break;
				}
				
				for(MapEdge neighbor : curr.getEdgeList()) {
					MapNode i = map.get(neighbor.getEndPoint());
					i.setDistanceToGoal(i.getLocation().distance(goal));
					if(!visited.contains(i)) {
						if(i.getDistance() > curr.distance + neighbor.getLength()) {
							i.setDistance(curr.distance + neighbor.getLength());
							parentMap.put(i, curr);
							PQ.add(i);
						}
					}
				}
			}
		}
		
		if(!found) return null;     // If goal node is not found, return null.
		
		List<GeographicPoint> path = constructPath(parentMap, startNode, goalNode);
		return path;

	}

	
	
	public static void main(String[] args)
	{
		
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		*/
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println(simpleTestMap.dijkstraNodes);
		System.out.println(simpleTestMap.aStarNodes);
		
		System.out.println();
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testMap.dijkstraNodes);
		System.out.println(testMap.aStarNodes);
		
		System.out.println();
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testMap.dijkstraNodes);
		System.out.println(testMap.aStarNodes);
		
		System.out.println();
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println(theMap.dijkstraNodes);
		System.out.println(theMap.aStarNodes);
		
		System.out.println();
		
		
	}
	
}
