/*
 * Driver.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 3 - Navigation
 * Group 53
 * This class drives the robot through a set of points. It does not
 * recognize obstacles in the way.
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Driver extends Thread implements UltrasonicController{
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 125;
	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double leftRadius, rightRadius, width;
	private Position[] positions;
	private int sensorDistance;
	private boolean isItNavigating;
	
	//Constructor
	public Driver(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		double leftRadius, double rightRadius, double width, Position[] positions){
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(600);
		}
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
		this.positions = positions;
		isItNavigating = false;
	}
	//thread
	public void run(){
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		//go through all of our positions, and travel to them.
		for(int i = 0; i < positions.length; i ++){
			travelTo(positions[i]);
		}
	}
	/*
	 * Moves the robot from it's current position to a final position passed in
	 * as a position object.
	 */
	public void travelTo(Position finalPos) {
		isItNavigating = true;
		Position currPos = odo.getPositionObject();

		//first we need to rotate the correct amount
		//then we'll move
		//get delta x and y, calculate hypotenuse and theta
		double deltaY = finalPos.getY()-currPos.getY();
		double deltaX = finalPos.getX()-currPos.getX();
		double hypotenuse = Math.sqrt(Math.pow(deltaY,2) + Math.pow(deltaX,2));
		double newTheta = Math.atan((double) deltaY/((double) deltaX))*360.0/(2*Math.PI);

		//Using deltaX, find out what the theta actually is (tan returns two possiblilites in our rangE)
		if(deltaX < 0){
			newTheta = 180.0 + newTheta;
		}
		//the amount to rotate
		double thetaToRotate = newTheta - currPos.getTheta();
		
		//limit it to the minimum angle
		if(thetaToRotate > 180){
			thetaToRotate = thetaToRotate-360.0;
		}else if(thetaToRotate < -180){
			thetaToRotate = thetaToRotate+360.0;
		}
		
		//turn this amount.
		turnTo(thetaToRotate);
		
		//now we move forward the hypotenuse distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, hypotenuse), true);
		rightMotor.rotate(convertDistance(rightRadius, hypotenuse), false);
		
		//since we're done, set isItNavigating to false;
		isItNavigating = false;
	}
	/*
	 * Turns the robot an amount theta.
	 */
	public void turnTo(double theta){
		//set the speeds of the motors and rotate them theta amount (turnTo turns an amount)
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(convertAngle(rightRadius, width, theta), false);
	}
	/*
	 * returns true if we're currently navigating.
	 */
	public boolean isNavigating(){
		return isItNavigating;
	}
	//Conversion methods.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	//Methods for the ultrasonic sensor (not currently being used in this version)
	@Override
	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		sensorDistance = distance;
		
	}
	@Override
	public int readUSDistance() {
		return this.sensorDistance;
	}
	
	@Override
	public double meToFinish(){
		return 0;
	}
	@Override
	public double blockToFinish(){
		return 0;
	}

}