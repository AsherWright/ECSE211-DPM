/*
 * Odometer.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;

import lejos.nxt.Motor;
import lejos.robotics.navigation.*;

/* To clockwise:
* 					0Deg:pos y-axis
* 							|
* 							|
* 							|
* 							|
* 270Deg:neg x-axis------------------90Deg:pos x-axis
* 							|
* 							|
* 							|
* 							|
* 					180Deg:neg y-axis
*/

public class Odometer extends Thread {
	// robot position
	private static  double x, y, theta;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	
	// lock object for mutual exclusion
	private static Object lock;

	// default constructor
	public Odometer() {



		lock = new Object();
	}

	/** 
	 * Starts the odometry thread
	 */
	public void run() {
		//set up variables
		long updateStart, updateEnd;
	    double leftTacho1,rightTacho1,leftTacho2,rightTacho2;
	    double leftLenth,rightLenth,D,average;
		double radiusW,radiusC,theta1;
		//initialize variables
		average= 0;
		x=0 ;
		y=0 ;
		theta = 0.0;
		leftTacho1=0;
		rightTacho1=0;
		radiusW = 2.65;
		radiusC = 12.25;
		
		while (true) {
			
			
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			

			//update tacho number and put them into calculation
			//we need minus left&rightTacho1 due to the updating method on x,y,theta 
			leftTacho2 = Motor.A.getTachoCount()-leftTacho1;
			rightTacho2 = Motor.B.getTachoCount()-rightTacho1;
			//store the tacho number in current loop
			leftTacho1 = Motor.A.getTachoCount();
			rightTacho1 = Motor.B.getTachoCount();
			//calculate the distance wheels traveled by
			leftLenth = leftTacho2*2*(Math.PI)*radiusW/360;
			rightLenth = rightTacho2*2*(Math.PI)*radiusW/360;
			average=(leftLenth+rightLenth)/2;
			//calculate theta (this part is too long so we break them into two parts)
			D = (leftTacho2-rightTacho2)*2*(Math.PI)*radiusW/360;
			theta1=D/(2*radiusC)/2/Math.PI*360;

			

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				//update x,y and theta
				if((theta>=360))
				{theta=theta-360;}
				else if(theta<=-360)
				{
					theta=theta+360;
				}
			theta = theta + theta1;
			x = x + average*Math.sin(theta/360*2*Math.PI);
			y = y + average*Math.cos(theta/360*2*Math.PI);




			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {

				}
			}
		}
	}

	/**
	 * Takes an array and set the first 3 positions to the current values of x,y, and theta
	 * 
	 * @param position The array where the values of x, y, and theta will be stored
	 */
	public void getPosition(double[] position) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
				position[0] = x;
				position[1] = y;
				position[2] = theta;
		}
	}
	
	/**
	 * Sets theta to a specific value
	 * 
	 * @param angle The new value for theta
	 */
	public void setTheta(double angle) {

		synchronized (lock) {
			theta = angle;
		}
	}

	/**
	 * Sets x to a specific value
	 * 
	 * @param newX The new value for x
	 */
	public void setX(double newX) {

		synchronized (lock) {
			x = newX;
		}
	}
	
	/**
	 * Sets y to a specific value
	 * 
	 * @param newY The new value for y
	 */
	public void setY(double newY) {

		synchronized (lock) {
			y = newY;
		}
	}
	
	/**
	 * Returns the current x position
	 * 
	 * @return Current value of x
	 */
	public static double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * Returns the current y position
	 * 
	 * @return Current value of y
	 */
	public static double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * Returns the current heading of the robot
	 * 
	 * @return Current value of theta
	 */
	public static double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**
	 * Sets the coordinates to a specific x, y, and theta
	 * 
	 * @param position The array where the new values of x, y, and theta will be stored
	 * @param update A boolean array of size 3 to control which values get updated i.e. if
	 * 				 update = {true, false, true}, x and theta will be updated but not y
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}


}