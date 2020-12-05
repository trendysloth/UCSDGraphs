package roadgraph;

import geography.GeographicPoint;

public class ComparableNode implements Comparable<ComparableNode>{
	GeographicPoint location;
	Double dist;
	
	public ComparableNode() {
		super();
	}
	
	public ComparableNode(GeographicPoint location, Double dist) {
		this.location = location;
		this.dist = dist;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public Double getDist() {
		return dist;
	}

	public void setDist(Double dist) {
		this.dist = dist;
	}

	@Override
	public int compareTo(ComparableNode o) {
		double result = (this.getDist() - o.getDist());
		return result < 0 ? -1 : result == 0 ? 0 : 1;
	}
}