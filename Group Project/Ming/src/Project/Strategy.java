/*
 * Strategy.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;

import lejos.nxt.*;
import Bluetooth.*;


public class Strategy {
	
	double angle, currentX, currentY;
	double d1, d2, w1;
	boolean loop = true, sawLine1, sawLine2;
	int count, line1, line2;
	private int LLightSensor = 480, RLightSensor = 435;
	
	private Odometer odo;
	private Navigation nav;
	private Transmission t;
	private StartCorner corner;
	private int cornerId;
	private LightSensor ls1, ls2;
	final double WHEEL_RADIUS = 2.75;
	
	public Strategy(Odometer odometer,UltrasonicSensor us1,UltrasonicSensor us2, Transmission t, LightSensor ls1, LightSensor ls2) {
		this.odo = odometer;
		this.nav = new Navigation(odo,us1,us2,t);
		this.t = t;
		this.ls1 = ls1;
		this.ls2 = ls2;
		this.corner = t.startingCorner;
		cornerId = corner.getId();
		this.d1 = t.d1 * 30.48;
		this.d2 = t.d2 * 30.48;
		this.w1 = t.w1 * 30.48;
	}
	
	
	public void defendStrategy()
	{
	 nav.turnTo(180);
	 if(cornerId == 1 || cornerId == 2) { nav.goBack(-20); }
	 else { nav.goBack(-10); }
		
	 Motor.C.rotate(-45);
	 try { Thread.sleep(3000); } catch(InterruptedException e) {}
	 Motor.C.rotate(45);	 
	}
	
	public void attackStrategy()
	{
		// 2 shooting angles
		nav.turnTo(0);
		
		
		Motor.C.rotate(-45);
		try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
		try { Thread.sleep(6500); } catch(InterruptedException e) {}
		
		turnToPoint((5*30.48)+20,10*30.48);		
		Motor.C.rotate(-45);
		try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
		try { Thread.sleep(6500); } catch(InterruptedException e) {} 
		
		nav.turnTo(-90);
	

		try { Thread.sleep(300); } catch(InterruptedException e) {} 
		nav.goForward(91.44 + 5);		
		turnToPoint((5*30.48)+4, 10*30.48);
		// trigger the touch sensor on the 2nd brick to signal that it is time to shoot (3 times)
		Motor.C.rotate(-45);
			try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
			try { Thread.sleep(6500); } catch(InterruptedException e) {}
		
		turnToPoint((5*30.48)+12, 10*30.48);
		Motor.C.rotate(-45);		
		try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
		try { Thread.sleep(7000); } catch(InterruptedException e) {} 
	

		turnToPoint((5*30.48)+16, 10*30.48);
		Motor.C.rotate(-45);	
			try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
			try { Thread.sleep(6500); } catch(InterruptedException e) {} 
		
		Sound.playTone(1200, 200);
		Motor.C.rotate(-45);	
			try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
			try { Thread.sleep(6500); } catch(InterruptedException e) {} 
		
		Motor.C.rotate(-45);	
			try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
			try { Thread.sleep(6500); } catch(InterruptedException e) {} 
			
		Motor.C.rotate(-45);	
			try { Thread.sleep(500); } catch(InterruptedException e) {} 
		Motor.C.rotate(45);		
			try { Thread.sleep(6500); } catch(InterruptedException e) {} 

		
	}
	

	private void turnToPoint(double targetX, double targetY) {
		currentX = odo.getX();
		currentY = odo.getY();
		angle =  Math.atan( (currentX-targetX)/ (currentY-targetY)  ) * (180/Math.PI);
		nav.turnTo(angle);
	}	
	
}