package roadgraph;
import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String roadType;
	private double distance;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, 
			String streetName, String roadType, double distance) {
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.distance = distance;
		this.roadType = roadType;
	}
	
	public GeographicPoint getStartPoint() {
		return this.start;
	}
	
	public GeographicPoint getEndPoint() {
		return this.end;
	}
	
	public String getStreetName() {
		return this.streetName;
	}
	
	public double getLength() {
		return this.distance;
	}
	
	public void setLength(double length) {
		this.distance = length;
	}

}
