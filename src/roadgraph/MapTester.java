package roadgraph;
import java.util.List;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapTester {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		/** Test Breadth First Search 
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		
		System.out.println("DONE.");
		
		System.out.println("Num nodes: " + theMap.getNumVertices()); // should be 9
		System.out.println("Num edges: " + theMap.getNumEdges()); // should be 22
		
		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(4.0, 2.0), new GeographicPoint(6.5, 0.0));
		for(GeographicPoint i : route) {
			System.out.println(i+ " --> ");
		}
		*/
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		for(GeographicPoint i : testroute) {
			System.out.println(i + " ---> ");
		}
		System.out.println();
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		for(GeographicPoint i : testroute2) {
			System.out.println(i + " ---> ");
		}
	}
}
