package roadgraph;
import geography.GeographicPoint;

@SuppressWarnings("serial")
public class Edge {

	/* Below instance variables describe the edge between 
	 * the geographicPoint contained within this class and 
	 * a given second geographicPoint */
	private GeographicPoint start, end;
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((end == null) ? 0 : end.hashCode());
		result = prime * result + ((start == null) ? 0 : start.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Edge other = (Edge) obj;
		if (end == null) {
			if (other.end != null)
				return false;
		} else if (!end.equals(other.end))
			return false;
		if (start == null) {
			if (other.start != null)
				return false;
		} else if (!start.equals(other.start))
			return false;
		return true;
	}

	/* Constructor */
	public Edge(GeographicPoint start, GeographicPoint end) {
		this.start = start;
		this.end = end;
	}
}
