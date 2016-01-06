package roadgraph;

public class EdgeProp {
	
	public Double getLength() {
		return length;
	}

	public String getType() {
		return type;
	}

	private Double length;
	private String type;
	
	public EdgeProp(Double length, String type) {
		this.length = length;
		this.type = type;
	}
}
