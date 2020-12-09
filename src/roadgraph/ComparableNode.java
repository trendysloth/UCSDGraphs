package roadgraph;

import geography.GeographicPoint;

public class ComparableNode implements Comparable<ComparableNode>{
	GeographicPoint location;
	Double dist;
	Double duration;
	
	public ComparableNode() {
		super();
	}
	
//	public ComparableNode(GeographicPoint location, Double dist) {
//		this.location = location;
//		this.dist = dist;
//	}

	public ComparableNode(GeographicPoint location, Double duration) {
		this.location = location;
		this.duration = duration;
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

	public Double getDuration() {
		return duration;
	}

	public void setDuration(Double duration) {
		this.duration = duration;
	}

	@Override
	public int compareTo(ComparableNode o) {
//		double result = (this.getDist() - o.getDist());
		double result = (this.getDuration() - o.getDuration());
		return result < 0 ? -1 : result == 0 ? 0 : 1;
	}
}