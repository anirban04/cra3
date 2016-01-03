package roadgraph;

@SuppressWarnings("serial")
public class Vertex extends geography.GeographicPoint implements Comparable<Vertex>{

	private double totalDist;
	
	public double getTotalDist() {
		return totalDist;
	}

	public void setTotalDist(double totalDist) {
		this.totalDist = totalDist;
	}

	public Vertex(double latitude, double longitude, double totalDist) {
		super(latitude, longitude);
		this.totalDist = totalDist;
	}
	
	public int compareTo(Vertex v) {
		if (this.totalDist > v.totalDist) {
			return 1;
		}
		else if (this.totalDist < v.totalDist) {
			return -1;
		}
		else
			return 0;
	}
}
