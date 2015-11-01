/*
 * LightLocalizer.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;

import lejos.nxt.*;
import Bluetooth.*;

public class LightLocalizer {
	
	private Odometer odo;
	private LightSensor ls1,ls2;
	private Navigation nav;
	Transmission t;
	private StartCorner corner;
	private int cornerId;
	
	public LightLocalizer(Odometer odo, LightSensor ls1,LightSensor ls2,UltrasonicSensor us1,UltrasonicSensor us2, Transmission t) {
		this.odo = odo;
		this.ls1 = ls1;
		this.ls2 = ls2;
		this.nav = new Navigation(odo,us1,us2,t);
		this.t = t;
		this.corner = t.startingCorner;
		ls1.setFloodlight(true);
		ls2.setFloodlight(true);
	}
	
	/**
	 * Localizes the robot using the light sensor: detects 2 gridlines and moves to their intersection
	 */
	public void doLocalization(int corner) {
		

		
		int count=0;
		double lightS1,lightS2;
		int timeSeeLine1=0,timeSeeLine2=0;
		boolean loop=true;

				Sound.beep();
		
				nav.rotate(95,95);
		
				try {	Thread.sleep(600);	} catch (InterruptedException e) {}
		
				// first localization phase: detect the gridline only with the left motor
				while(loop)
				{
					
					// once the line is detected, stop the robot
					lightS1 = ls1.getNormalizedLightValue();
					if( (lightS1<480) && (timeSeeLine1==0)  )
					{
						++count;
						Sound.beep();
						Motor.A.stop();
						timeSeeLine1=1;
					}
					
					// after stopping the robot, go back 12.5cm (distance from the light sensor to the wheels)
					if(count==1)
					{
						odo.setTheta(0);
						nav.goBack(-12.5);
						odo.setY(0);

						loop= false;
					}
					
				}
				
				nav.turnTo(90);
				loop=true;
				nav.rotate(75,75);
				timeSeeLine1=0;
				timeSeeLine2=0;
				count=0;
				
				// second localization phase: detect gridline with both sensors after turning 90 degrees clockwise 
				while(loop)
				{
						
					lightS1 = ls1.getNormalizedLightValue();
					lightS2 = ls2.getNormalizedLightValue();
					// whichever LS detects the line first will stop in order to let the second LS "catch up"
					if( (lightS1<470) && (timeSeeLine1==0)&&(lightS2<450) && (timeSeeLine2==0)  )
					{
						count=2;
						Sound.beep();
						nav.stopMotor();
						timeSeeLine1=1;
						timeSeeLine2=1;
					}
					
					if( (lightS1<470) && (timeSeeLine1==0)  )
					{
						++count;
						Sound.beep();
						Motor.A.stop();
						timeSeeLine1=1;
					}
					
					
					
					if( (lightS2<450) && (timeSeeLine2==0) )
					{
						++count;
						Sound.beep();
						Motor.B.stop();
						timeSeeLine2=1;
					}
					
					
					// once both lines are detected, go back 12.5cm
					if(count==2)
					{

						odo.setTheta(90);
						nav.goBack(-12.5);
						odo.setX(0);

						loop= false;

					}
					

				}
			
				
				nav.turnTo(0);
				loop=true;
				nav.rotate(75,75);
				timeSeeLine1=0;
				timeSeeLine2=0;
				count=0;
				
				// third localiation phase: detect gridline with both sensors after turning 90 degrees counterclockwise
				while(loop)
				{
					// whichever LS detects the line first will stop in order to let the second LS "catch up"
						
					lightS1 = ls1.getNormalizedLightValue();
					lightS2 = ls2.getNormalizedLightValue();
					if( (lightS1<470) && (timeSeeLine1==0)&&(lightS2<450) && (timeSeeLine2==0)  )
					{
						count=2;
						Sound.beep();
						nav.stopMotor();
						timeSeeLine1=1;
						timeSeeLine2=1;
					}
					
					if( (lightS1<470) && (timeSeeLine1==0)  )
					{
						++count;
						Sound.beep();
						Motor.A.stop();
						timeSeeLine1=1;
					}
					
					
				
					if( (lightS2<450) && (timeSeeLine2==0) )
					{
						++count;
						Sound.beep();
						Motor.B.stop();
						timeSeeLine2=1;
					}
					
					// once both lines are detected, go back 12.5cm
					
					if(count==2)
					{
						odo.setTheta(0);
						nav.goBack(-12.5);
						odo.setY(0);

						loop= false;
					}
					
				}
				
			// localization is complete.
			// set the odometer to the robot's current position, depending on the parameter passed in through Bluetooth
			cornerId = corner;
			boolean[] updateAll = {true,true,true};
			
			if(cornerId == 1) {
				odo.setPosition(new double[] {0,0,0}, updateAll);
			}
			else if(cornerId == 2) {
				odo.setPosition(new double[] {304.8,0,-90}, updateAll);
			}
			else if(cornerId == 3) {
				odo.setPosition(new double[] {304.8,304.8,-180}, updateAll);
			}
			else if(cornerId == 4) {
				odo.setPosition(new double[] {0,304.8, 90}, updateAll);
			}
			else {
				Sound.buzz();
			}
				
		}
			

			
}

