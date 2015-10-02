
public class Position {

	private double x;
	private double y;
	private double theta;
	
	public Position(double x, double y, double theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}	
	public void setPosition(double x, double y, double theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getTheta() {
		return theta;
	}

	public void setTheta(double theta) {
		this.theta = theta;
	}
	
	public String toString(){
		return "(" + x + "," + " y) : " + theta;
	}
}
