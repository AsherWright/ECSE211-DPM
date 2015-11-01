/*
 * FinalProject.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;


import lejos.nxt.*;
import java.util.*;
import Bluetooth.*;

public class FinalProject extends Thread {


	
	public static void main(String[] args) {
		int buttonChoice;
		double d1 = 0, d2 = 0, w1 = 0;
		int travelX, travelY;
		boolean loop;
		double angle;

		// some objects that need to be instantiated
		Odometer odometer = new Odometer();
		Random rand = new Random();
		UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S1);
		UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S2);
		LightSensor ls1 = new LightSensor(SensorPort.S3);
		LightSensor ls2 = new LightSensor(SensorPort.S4);
		BluetoothConnection conn = new BluetoothConnection();
		Transmission t = conn.getTransmission();
		Strategy strat = new Strategy(odometer,us1,us2,t,ls1,ls2);
		Navigation nav= new Navigation(odometer,us1,us2,t);
		StartCorner corner;
		int cornerId=0;
		int roleId=0;
		PlayerRole role;		
		
		travelX = 0;
		travelY = 0;
		loop = true;	
		boolean init = false;
		
		do {
			
			LCD.clear();

			if (t == null) {
				LCD.drawString("Failed to read transmission", 0, 5);				
			} 
			
			else {
				//play some sound before formal competition start
				Sound.setVolume(80);
				int a = rand.nextInt(400);
				Sound.playTone(400 + a, 200);
				Sound.pause(100);
				Sound.playTone(600 + a, 200);
				Sound.pause(100);
				Sound.playTone(800 + a, 200);
				Sound.pause(100);
				Sound.playTone(1000 + a, 200);
				Sound.pause(100);
				Sound.playTone(800 + a, 200);
				
				corner = t.startingCorner;
				role = t.role;
				d1 = t.d1 * 30.48;
				d2 = t.d2 * 30.48;
				w1 = t.w1 * 30.48;	
				
				cornerId = corner.getId();
				roleId = role.getId();
				
				// print out the transmission information
				conn.printTransmission();
				init = true;
			}	
			
		} while (!init);
		

		
		// manual coordinate entry		(for testing)
//		while (loop) {
//			LCD.clear();
//			LCD.drawString("" + travelX, 0, 1);
//			LCD.drawString("" + travelY, 0, 2);
//			
//			buttonChoice = Button.waitForPress();
//			
//			if(buttonChoice == Button.ID_LEFT) {
//				travelX+=30.48;
//			}
//			else if(buttonChoice == Button.ID_ENTER) {
//				travelY+=30.48;
//			}
//			else if(buttonChoice == Button.ID_ESCAPE) {
//				loop = false;
//			}
//		}

		
			
			
			// manual parameter entry(for testing)
//			roleId = 2;		
//		    cornerId = 4;
//		    d1 = 3*30.48;
//		    d2 = 3*30.48;
//		    w1 = 3*30.48;
		    	
		
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
			odometer.start();
			odometryDisplay.start();
			

			
						
			// perform the ultrasonic localization
			USLocalizer usl = new USLocalizer(odometer, us1, us2, t);
			usl.doLocalization();
			// perform the light sensor localization
			LightLocalizer lsl = new LightLocalizer(odometer, ls1, ls2, us1, us2, t);
			lsl.doLocalization(cornerId);	
			
			nav.getToField(cornerId);
			
			// manual
//			travelX = 150;
//			travelY = 240;			
//			nav.travelTo(travelX, travelY, true);
//			while(nav.jump)
//			{
//				nav.travelTo(travelX, travelY, true);
//				}
			
			// auto
			// attacker role
			if(roleId == 1) {
				if(cornerId == 3 || cornerId == 4)
				{
					nav.turnTo(186);
					odometer.setTheta(180);
				}
				if(cornerId == 1 || cornerId == 2)
				{
					nav.turnTo(3);
					odometer.setTheta(0);
				}
				nav.travelTo(5*30.48, (d2-30.48-15.24), true, false);
				while(nav.jump)
				{
					nav.travelTo(5*30.48, (d2-30.48-15.24), true, false);
				}
				strat.attackStrategy();
			}
			// defender role
			else if(roleId == 2) {
				double midDistance;
				if(cornerId == 1 || cornerId == 2) { midDistance = 152.4-14; }
				else { midDistance = 152.4-20; }
				if (d2 == 60.96) {
				
					nav.travelTo(midDistance, 304.8-30.48, false, false);
					while(nav.jump)
					{
						nav.travelTo(midDistance, 304.8-30.48, false, false);
					}
					strat.defendStrategy();
				}
				else {
					nav.travelTo(midDistance, 304.8-30.48-20, false, false);
					while(nav.jump)
					{
						nav.travelTo(midDistance, 304.8-30.48-20, false, false);
					}
					strat.defendStrategy();
				}
			}
			//no role specified, default is attacker
			else {
				Sound.buzz();
			}					
		
		while (Button.waitForPress() != Button.ID_ENTER);
		System.exit(0);
	}

}
