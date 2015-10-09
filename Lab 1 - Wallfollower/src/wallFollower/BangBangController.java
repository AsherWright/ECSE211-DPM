package wallFollower;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh, closestAllowedDistance;
	private final int FILTER_OUT = 8;
	//any measurement above 80 is considered a max distance measurement (for the filter)
	private final int MAXDISTANCETHRESHOLD = 80;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int filterControl;
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh, int closestAllowedDistance) {
		//Default Constructor
		//set variables
		this.bandCenter = bandCenter; //the middle location of the band that our robot SHOULD be in
		this.bandwidth = bandwidth; //the width of the band that the robot SHOULD be in.
		this.motorLow = motorLow; //the lower speed for the wheel (deg/s)
		this.motorHigh = motorHigh; //the upper speed for the wheel (deg/s)
		this.leftMotor = leftMotor; //LEFT MOTOR
		this.rightMotor = rightMotor; //RIGHT MOTOR
		this.closestAllowedDistance = closestAllowedDistance; //the closest difference before the robot will stop and rotate in place
		
		//set speed of both motors to high
		leftMotor.setSpeed(motorHigh);				
		rightMotor.setSpeed(motorHigh);
		//start both motors forward (start moving robot)
		leftMotor.forward();
		rightMotor.forward();
		//start the filterControl at 0
		filterControl = 0;
	}
	
	@Override
	public void processUSData(int distanceMeasurement) {
		// rudimentary filter - toss out invalid samples corresponding to null signal.
				// This filter was taken from the PController code
				//
				// if we're reading past our max distance threshold and we haven't read it
				// filterOut times, then ignore this measurement!
				if (distanceMeasurement > MAXDISTANCETHRESHOLD && filterControl < FILTER_OUT) {
					// bad value, do not set the distance var, however do increment the filter value
					filterControl ++;
				} else if (distanceMeasurement > MAXDISTANCETHRESHOLD){ //if we've read max after this long
					// true measurement, so set it. 
					this.distance = distanceMeasurement;
				} else {
					// distance went below our threshold, therefore reset everything.
					filterControl = 0;
					this.distance = distanceMeasurement;
				}
		
		//set local distance variable to passed in value
		this.distance = distanceMeasurement;
		//if our robot is closer to the wall than the lowest point of the allowed band (bandCenter-bandwidth)
		if(distance < bandCenter-bandwidth){
			//if we are closer than our closest allowed distance, we will rotate in spot (spin right backwards)
			if(distance <= closestAllowedDistance){
				leftMotor.setSpeed(motorLow);				//set the speed to the lower speed (both motors)
				rightMotor.setSpeed(motorLow);				
				leftMotor.forward();						// Spin left motor forward
				rightMotor.backward();						// spin right motor BACKWARDS. This will rotate in space
			}else{	//if we aren't too close, we just try to realign to get back into our allowed BAND.
				leftMotor.setSpeed(motorHigh);				// set left high
				rightMotor.setSpeed(motorLow);				// set right low (this means we'll move to the right, away from wall)
				//move both motors forward (to move robot forward).
				leftMotor.forward();						
				rightMotor.forward();	
			}
		//if our robot is farther than the highest point of the allowed band (bandCenter + bandwidth)
		}else if (distance > bandCenter + bandwidth){
			leftMotor.setSpeed(motorLow);				// set left low
			rightMotor.setSpeed(motorHigh);				// set right high (this will move robot closer to wall)
			//spin both motors forward
			leftMotor.forward();
			rightMotor.forward();
		//if our robot is inside the band, we just want to move forward. 
		}else{
			//set both motor speeds to high, 
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorHigh);
			//spin them (move robot forward).
			leftMotor.forward();
			rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
