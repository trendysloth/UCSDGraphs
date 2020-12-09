/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
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
 * @param
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> vertices;
	private HashMap<GeographicPoint, Double> dist;
	private HashMap<GeographicPoint, Double> duration;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<>();
		dist = new HashMap<>();
		duration = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		Integer numOfEdges = 0;
		for (MapNode mapNode : vertices.values()) {
			numOfEdges = numOfEdges + mapNode.getEdges().size();
		}
		return numOfEdges;
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
		// TODO: Implement this method in WEEK 3
		if (location == null || vertices.containsKey(location)) {
			return false;
		}
		MapNode newNode = new MapNode();
		newNode.setLocation(location);
		vertices.put(location, newNode);
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

		//TODO: Implement this method in WEEK 3
		if (from == null || to == null || roadName == null || roadType == null || length < 0) {
			
			throw new IllegalArgumentException();
		}
		
		MapNode currNode = vertices.get(from);
		if (currNode == null) {
			throw new IllegalArgumentException();
		}
		
		List<MapEdge> edges = currNode.getEdges();
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		edges.add(edge);
		currNode.setEdges(edges);
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
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
//		nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		return constructPath(start, goal, parentMap);
	}
	

	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
		HashSet<GeographicPoint> visited = new HashSet<>();
		Queue<GeographicPoint> toExplore = new LinkedList<>();
		toExplore.add(start);

		while (!toExplore.isEmpty()) {
			GeographicPoint current = toExplore.remove();
			nodeSearched.accept(current);
			if (current.equals(goal)) {
				return true;
			}
			MapNode currentNode = vertices.get(current);
			List<MapEdge> edges = currentNode.getEdges();
			
			for (MapEdge edge : edges) {
				GeographicPoint currNeighbor = edge.getEnd();
				if (!visited.contains(currNeighbor)) {
					visited.add(currNeighbor);
					toExplore.add(currNeighbor);
					parentMap.put(currNeighbor, current);
				}
			}
		}

		return false;
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
										  GeographicPoint goal, 
										  Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.s
//		nodeSearched.accept(next.getLocation());
		Integer numOfNodesExplored = 0;
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		HashSet<ComparableNode> visited = new HashSet<>();
		PriorityQueue<ComparableNode> q = new PriorityQueue<>();
		q.add(new ComparableNode(start, 0.0));
		
		for (GeographicPoint point : vertices.keySet()) {
			if (point.equals(start)) {
//				dist.put(point, 0.0);
				duration.put(point, 0.0);
			} else {
//				dist.put(point, Double.MAX_VALUE);
				duration.put(point, Double.MAX_VALUE);
			}
		}
		while (!q.isEmpty()) {
			ComparableNode node = q.remove();
			numOfNodesExplored++;
			if (!visited.contains(node)) {
				visited.add(node);
				if (node.getLocation().equals(goal)) {
					System.out.println(numOfNodesExplored);
					return constructPath(start, goal, parentMap);
				}
			}
			
//			Double currentDist = dist.get(node.getLocation());
			Double currentDuration = duration.get(node.getLocation());
			GeographicPoint current = node.getLocation();
			MapNode mapNode = vertices.get(current);
			
			for (MapEdge edge : mapNode.getEdges()) {
				GeographicPoint currNeighbor = edge.getEnd();
				String roadType = edge.getRoadType();
				Double speedLimit;
				if (roadType.equals("primary")) {
					speedLimit = 65.0;
				} else if (roadType.equals("secondary")) {
					speedLimit = 45.0;
				} else if (roadType.equals("residential")) {
					speedLimit = 30.0;
				} else {
					speedLimit = 15.0;
				}
//				Double currNeighborDist = dist.get(currNeighbor);
//				Double newCurrNeighborDist = currentDist + edge.getDistance();
//				if (newCurrNeighborDist < currNeighborDist) {
//					dist.put(currNeighbor, newCurrNeighborDist);
//					parentMap.put(currNeighbor, current);
//					q.add(new ComparableNode(currNeighbor, newCurrNeighborDist));
//				}
				Double currNeighborDuration = duration.get(currNeighbor);
				Double newCurrNeighborDuration = currentDuration + edge.getDistance() / speedLimit;
				if (newCurrNeighborDuration < currNeighborDuration) {
					duration.put(currNeighbor, newCurrNeighborDuration);
					parentMap.put(currNeighbor, current);
					q.add(new ComparableNode(currNeighbor, newCurrNeighborDuration));
				}

			}
		}
		System.out.println(numOfNodesExplored);
		return null;
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
		// TODO: Implement this method in WEEK 4
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		Integer numOfNodesExplored = 0;
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		HashSet<ComparableNode> visited = new HashSet<>();
		PriorityQueue<ComparableNode> q = new PriorityQueue<>();
		q.add(new ComparableNode(start, 0.0));
		
		for (GeographicPoint point : vertices.keySet()) {
			if (point.equals(start)) {
//				dist.put(point, 0.0);
				duration.put(point, 0.0);
			} else {
//				dist.put(point, Double.MAX_VALUE);
				duration.put(point, Double.MAX_VALUE);
			}
		}
		
		while (!q.isEmpty()) {
			ComparableNode node = q.remove();
			numOfNodesExplored++;
			if (!visited.contains(node)) {
				visited.add(node);
				if (node.getLocation().equals(goal)) {
//					System.out.println(numOfNodesExplored);
					return constructPath(start, goal, parentMap);
				}
			}
			
//			Double currentDist = dist.get(node.getLocation());
			Double currentDuration = duration.get(node.getLocation());
			GeographicPoint current = node.getLocation();
			MapNode mapNode = vertices.get(current);
			
			for (MapEdge edge : mapNode.getEdges()) {
				GeographicPoint currNeighbor = edge.getEnd();

				String roadType = edge.getRoadType();
				Double speedLimit;
				if (roadType.equals("primary")) {
					speedLimit = 65.0;
				} else if (roadType.equals("secondary")) {
					speedLimit = 45.0;
				} else if (roadType.equals("residential")) {
					speedLimit = 30.0;
				} else {
					speedLimit = 15.0;
				}

				Double currHeruistics = duration.get(currNeighbor);
				Double newCurrNeighborHeruistics = currentDuration + edge.getDistance() / speedLimit + currNeighbor.distance(goal);

				if (newCurrNeighborHeruistics < currHeruistics) {
					duration.put(currNeighbor, newCurrNeighborHeruistics);
					parentMap.put(currNeighbor, current);
					q.add(new ComparableNode(currNeighbor, newCurrNeighborHeruistics));
				}
			}
		}
//		System.out.println(numOfNodesExplored);
		return null;
	}

	
	private static List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint current = goal;
		while (!current.equals(start)) {
			path.addFirst(current);
			current = parentMap.get(current);
		}
		path.addFirst(start);
		return path;
	}
	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
}
