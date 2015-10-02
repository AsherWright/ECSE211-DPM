

/*
 * SquareDriver.java
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Driver extends Thread{
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 125;
	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double leftRadius, rightRadius, width;
	private Position[] positions;
	
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
	}
	public void run(){
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		for(int i = 0; i < positions.length; i ++){
			driveToPosition(positions[i]);
		}
	}
	private void driveToPosition(Position finalPos) {
		Position currPos = odo.getPositionObject();

		//first we need to rotate the correct amount
		
		double deltaY = finalPos.getY()-currPos.getY();
		double deltaX = finalPos.getX()-currPos.getX();
		double hypotenuse = Math.sqrt(Math.pow(deltaY,2) + Math.pow(deltaX,2));
		double newTheta = Math.atan((double) deltaY/((double) deltaX))*360.0/(2*Math.PI);
		//System.out.println("new theta: " + newTheta);
		double thetaToRotate = newTheta - currPos.getTheta();
		
		if(thetaToRotate > 180){
			thetaToRotate = thetaToRotate-360.0;
		}else if(thetaToRotate < -180){
			thetaToRotate = thetaToRotate+360.0;
		}
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(leftRadius, width, thetaToRotate), true);
		rightMotor.rotate(convertAngle(rightRadius, width, thetaToRotate), false);
		//System.out.println("Angle to rotate: " + convertAngle(rightRadius, width, thetaToRotate));
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, hypotenuse), true);
		rightMotor.rotate(convertDistance(rightRadius, hypotenuse), false);
		

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(leftRadius, width, finalPos.getTheta() -thetaToRotate), true);
		rightMotor.rotate(convertAngle(rightRadius, width, finalPos.getTheta() -thetaToRotate), false);
	}
	//Conversion methods.
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}