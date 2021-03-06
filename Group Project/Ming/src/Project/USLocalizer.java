/*
 * USLocalizer.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;


import lejos.nxt.Motor;
import Bluetooth.*;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
		public enum LocalizationType { FALLING_EDGE, RISING_EDGE };


		private Odometer odo;
		private UltrasonicSensor us;

		private LocalizationType locType;
		private static double angle,angleA, angleB;
		public static int minDistance=255;
		Navigation nav;
	
	
	/**
	 * The default constructor
	 * 
	 * @param odo The robot's odometer
	 * @param us The robot's left ultrasonic sensor
	 */
	public USLocalizer(Odometer odo, UltrasonicSensor us1,UltrasonicSensor us2, Transmission t) {
		this.odo = odo;
		this.us = us1;
		this.nav = new Navigation(odo,us1,us2,t);
		
		// switch off the ultrasonic sensor
		us.off();
	}
	
	/**
	 * Localizes the robot using the ultrasonic sensor. Clocks the angle at which the sensor detects each wall and calculates 0 degrees
	 */
	public void doLocalization() {
		
		
		angleA=0;
		angleB=0;
		int UCdistance,expmtDistance;
		expmtDistance = 38;
		int count=0;
		boolean facingToWall=true;
		
			// rotate the robot until it sees no wall
			nav.rotate(165,-165);
			facingToWall = true;
			
			while(facingToWall)
			{
				UCdistance = getFilteredData();
				if(UCdistance>expmtDistance)
				{
					count++;
				}
				if(UCdistance>expmtDistance&&count>=3)
				{
					facingToWall = false;
					count=0;
					}
			}	
			
			
			sleep(3000);
			
			// keep rotating until the robot sees a wall, then latch the angle
			while(!facingToWall)
			{
				UCdistance = getFilteredData();	
				if(UCdistance<expmtDistance)
				{
					count++;
				}
				
				if(UCdistance<expmtDistance&&count>=3)
				{
					facingToWall = true;
					angleA = odo.getTheta();
					Sound.beep();
					nav.stopMotor();
					count=0;
					}
			}	


			// switch direction and wait until it sees no wall
		    nav.rotate(-165,165);
			while(facingToWall)
			{
				UCdistance = getFilteredData();	
				
				if(UCdistance>expmtDistance)
				{
					count++;
				}
				
				if(UCdistance>expmtDistance&&count>=3)
				{
					facingToWall = false;
					count=0;
				}
			}	
			
			
			// keep rotating until the robot sees a wall, then latch the angle
			sleep(3000);
			
			while(!facingToWall)
			{
				UCdistance = getFilteredData();		
				if(UCdistance<expmtDistance)
				{
					count++;
				}
				
				if(UCdistance<expmtDistance&&count>=3)
				{
					facingToWall = true;
					angleB = odo.getTheta();
					Sound.beep();
					nav.stopMotor();
					}
			}	

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			// we slightly correct the angle '45' to '44' based on the error we measured
			angle = 45 - (angleA - angleB)/2;
			
			// update the odometer position (example to follow:)
			odo.setTheta(angle);
			nav.turnTo(0);


		 
	}
	
	/**
	 * Reads data from the ultrasonic sensor and filters it to prevent false 255 values
	 * 
	 * @return The filtered data
	 */
	private int getFilteredData() {
		int distance;
		
		// do a ping
		us.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		distance = us.getDistance();
		
		if(minDistance>distance)
		{minDistance = distance;}
		
		if (distance > 55)
		{distance = 55;}
	
		if (distance > 255)
		{distance = 255;}
		
		return distance;
	}
	
	
	/**
	 * Thread sleeps for a specified time
	 * 
	 * @param time The time the thread will sleep (in ms)
	 */
	private void sleep(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
		}
	}


	



}