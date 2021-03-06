/*
 * Driver.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding Objects
 * Group 53
 * Given that the robot is facing 90 degrees at 0,0, scans the area for blocks,
 * and heads towards the first block it finds. It then determines the type of block,
 * and moves it to the end point if it is a styrofoam.
 */
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class Driver extends Thread {
	private static final int ROTATESPEED = 30;
	private static final int FORWARDSPEED = 50;
	
	//class variables
	//for the ultrasonic data
	private SampleProvider usSensor;
	private float[] usData;
	//The motors for moving the robot and for moving the arm to capture the block
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3LargeRegulatedMotor armMotor1;
	EV3LargeRegulatedMotor armMotor2;
	//odometer, navigator, and blockdetector
	Odometer odo;
	Navigation navi;
	BlockDetector detector;
	/*
	 * constructor. We need the odometer (for position info), the detector (for block detection), and the US data
	 */
	public Driver(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor armMotor1,EV3LargeRegulatedMotor armMotor2, Odometer odo, BlockDetector detector, SampleProvider usSensor, float[] usData, Navigation navi){
		//get all incoming variables
		this.usData = usData;
		this.usSensor = usSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odo;
		this.navi = navi;
		this.detector= detector;
		this.armMotor1 = armMotor1;
		this.armMotor2 = armMotor2;
	}
	
	/*
	 * Finds the block on the field, grabs it, and returns it.
	 */
	public void run(){
		//begin spinning clockwise to scan for blocks
		leftMotor.setSpeed(ROTATESPEED);
		rightMotor.setSpeed(ROTATESPEED);
		leftMotor.forward();
		rightMotor.backward();
		//this variable makes sure we only do a close-adjustment once
		boolean haveAdjustedClose = false;
		while(true){
			//get the distance value
			double USDistance = getFilteredUSData();
			/*
			 * if we are at a block, stop.
			 * if we are really close to a block, but kind of off, adjust slightly (do this only once)
			 * if we see a block, move towards it
			 * if we do not see a block, turn.
			 */
			if(USDistance < 5){
				leftMotor.stop(true);
				rightMotor.stop(true);
				break;
			}else if(USDistance <22 && haveAdjustedClose == false){
				//this turns a set amount
				leftMotor.rotate(80,true);
				rightMotor.rotate(-80,false);
				Sound.buzz();
				//we've made the adjustment
				haveAdjustedClose = true;
		
				
			}else if(USDistance < 80){
				//move forward
				leftMotor.setSpeed(FORWARDSPEED);
				rightMotor.setSpeed(FORWARDSPEED);
				leftMotor.forward();
				rightMotor.forward();
			}else if(USDistance > 80){
				//spin
				leftMotor.setSpeed(ROTATESPEED);
				rightMotor.setSpeed(ROTATESPEED);
				leftMotor.forward();
				rightMotor.backward();
			}
			
		}
		//We should wait before reading the detector (so it settles)
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		/*
		 * Check to see what we are reading for an object.
		 * we beep once for a block and twice for the wooden block.
		 */
		if(detector.isReadingBlock()){
			Sound.beep();
			//since it is a block, bring down the arms, and travel to the end position
			bringDownArms();
			navi.travelTo(65, 65);
			//then beep thrice (as specified in document)
			Sound.beep();
			Sound.beep();
			Sound.beep();
		}else{
			Sound.beep();
			Sound.beep();
		}
		
	}
	/*
	 * brings down the arms of the robot to capture the block.
	 */
	private void bringDownArms(){
		armMotor1.setSpeed(20);
		armMotor2.setSpeed(20);
		armMotor1.rotate(110, true);
		armMotor2.rotate(110, false);
	}
	/*
	 * Gets the ultrasonic sensor data from the US sensor
	 */
	private float getFilteredUSData() {
		usSensor.fetchSample(usData, 0);
		float distance = (int)(usData[0]*100.0);
		float result = 0;
		if (distance > 200){
			// true 255, therefore set distance to 255
			result = 200; //clips it at 50
		} else {
			// distance went below 255, therefore reset everything.
			result = distance;
		}
		//lastDistance = distance;
		return result;
	}

}
