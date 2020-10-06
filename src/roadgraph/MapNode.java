package roadgraph;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import geography.GeographicPoint;

public class MapNode implements Comparable{
	private GeographicPoint location;
	private List<MapEdge> edgeList;
	private int edgeNum;
	double distance;
	double distanceToGoal;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		edgeList = new ArrayList<MapEdge>();
		this.edgeNum = 0;
		this.distance = 0;
		this.distanceToGoal = 0;
	}
	
	public void addEdge(MapEdge edge) {
		edgeList.add(edge);
		edgeNum++;
	}
	
	public int getEdgeNum() {
		return this.edgeNum;
	}
	
	public void printEdges() {
		for(MapEdge i : edgeList) {
			System.out.println(i);
		}
	}
	
	public List<MapEdge> getEdgeList() {
		List<MapEdge> edgeListCopy = edgeList;
		return edgeListCopy;
	}
	
	public GeographicPoint getLocation() {
		return this.location;
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	public double getDistanceToGoal() {
		return this.distanceToGoal;
	}
	
	public void setDistanceToGoal(double distance) {
		this.distanceToGoal = distance;
	}

	@Override
	public int compareTo(Object x) {
		// TODO Auto-generated method stub
		MapNode m = (MapNode)x;
		return ((Double)this.getDistance()).compareTo(m.getDistance());
	}
	
	
	
	
}
