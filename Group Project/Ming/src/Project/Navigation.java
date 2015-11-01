/*
 * Navigation.java
 * Team 5
 * DPM - Fall 2011
 */

package Project;

import lejos.nxt.*;
import Bluetooth.*;
import lejos.util.Delay;

public class Navigation {
	final static int FAST = 300, SLOW = 195, SLOWER = 175, SLOWEST = 120, ACCELERATION = 200;
	final static double DEG_ERR = 10.0, CM_ERR = 1.0;
	final double WHEEL_RADIUS = 2.65, WIDTH = 24.5, BRICKWIDTH = 30.48;
	private double FIELD_ANGLE = 62;
    
	private Odometer odometer;
	private UltrasonicSensor us1, us2;
	private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	private final LightSensor light1 = new LightSensor(SensorPort.S3);
	private final LightSensor light2 = new LightSensor(SensorPort.S4);
	private final Delay wait = new Delay();
	private double deltaTheta;
	private double currentX, currentY, deltaX, deltaY, theta;
	private double distance;
	private double goalx1, goalx2, goaly1, goaly2;
	private double destX, destY;
	private double line1, line2;
	private int distance1, distance2;
	private int count;
	private int sawLine1, sawLine2;
	private Transmission t;
	private double THETA;
	private int cornerId;
	private StartCorner corner;
	private PlayerRole role;
	private int roleId;
	boolean jump = false, loop = true;
	boolean drivingToY = false;
	private int USdistance = 40;
	private int LLightSensor = 468, RLightSensor = 435;
    boolean Dontturnback=false;
	private final int FILTER_OUT = 4, FILTER_CLEAN = 15;

	// constructor
	public Navigation(Odometer odo, UltrasonicSensor us1, UltrasonicSensor us2,
			Transmission t) {
		this.odometer = odo;
		this.us1 = us1;
		this.us2 = us2;
		this.t = t;
		this.role = t.role;
		this.corner = t.startingCorner;
		roleId = role.getId();
		cornerId = corner.getId();
		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * Sets speed of both motors simultaneously
	 * 
	 * @param lSpd
	 *            Speed of left motor (float)
	 * @param rSpd
	 *            Speed of right motor (float)
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * Sets speed of both motors simultaneously
	 * 
	 * @param lSpd
	 *            Speed of left motor (int)
	 * @param rSpd
	 *            Speed of right motor (int)
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * Sets both motors to float mode (no resistance to rotation)
	 */
	public void setFloat() {
		stopMotor();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/**
	 * Navigates to a specific point on the coordinate plane. Avoids obstacles
	 * along the way.
	 * 
	 * @param x
	 *            X-coordinate you want to travel to
	 * @param y
	 *            Y-coordinate you want to travel to
	 */
	public void travelTo(double x, double y, boolean UCactive, boolean LSactive) {
		// this is used for return isNavigating method
		destX = x;
		destY = y;
		if (drivingToY) {
			defineRouteY(x, y);
		} else {
			defineRoute(x, y);
		}

		double x0, y0, X, Y, angle, thetaOdo, theta1, theta3;
		double x2, y2, theta2, count, count1;
		double position;
		double value;
		double newValue;
		double Antioverturn1, thetarecord;
		boolean FacingToX = false;
		boolean Overturn = false;
		boolean UCactive1 = true;
		int Overturnangle = 30;
		int filter1 = 0;
		int filter2 = 0;
		int filter1clean = 0, filter2clean = 0;
		// initialize variables and get starting point from odometer
		count = 0;
		count1 = 0; // Counter used for obstacle detection
		x0 = Odometer.getX();
		y0 = Odometer.getY();
		thetaOdo = Odometer.getTheta();

		// calculate the angle we need to turn
		X = goalx1 - x0;
		Y = goaly1 - y0;
		theta1 = Math.atan2(X, Y) / (2 * Math.PI) * 360;

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}

		// turnTo that direction
		turnTo(theta1);

		Motor.A.setSpeed(SLOW);
		Motor.B.setSpeed(SLOW);
		
		if(roleId == 2) {
			Motor.A.setSpeed(FAST);
			Motor.B.setSpeed(FAST);
		}

		Motor.A.forward();
		Motor.B.forward();
		THETA = odometer.getTheta();

		if (((THETA < 7) && (THETA > -7)) || ((THETA < 187) && (THETA > 173))
				|| ((THETA < -173) && (THETA > -187))) {
			FacingToX = false;
		} else if (((THETA < 97) && (THETA > 83))
				|| ((THETA < -83) && (THETA > -97))) {
			FacingToX = true;
		}

		// keep going until the robot reaches the target destination
		while (count == 0) {
			x2 = odometer.getX();
			y2 = odometer.getY();
			theta2 = odometer.getTheta();

			// grid-snapping code(both Ultrasonic sensor avoidance and grid snapping is in this method)
			if (LSactive) {
				
				line1 = light1.getNormalizedLightValue();
				line2 = light2.getNormalizedLightValue();

				// robot is not straight, it is a bit to the right
				if (line1 <= LLightSensor && line2 > RLightSensor) {
					Sound.beep();
					leftMotor.stop();
					thetarecord = odometer.getTheta();
					rightMotor.backward();
					wait.msDelay(1000);
					rightMotor.forward();
					Antioverturn1 = odometer.getTheta();
					while ((line2 > RLightSensor) && (!Overturn)) {
						line2 = light2.getNormalizedLightValue();
						if (Math.abs(odometer.getTheta() - Antioverturn1) >= Overturnangle) {
							Overturn = true;
						}
					}
					rightMotor.stop();
					if (Overturn) {
						rightMotor.rotate(-(45 + Overturnangle));
						Overturn = false;
						odometer.setTheta(thetarecord);
					} else {
						LSupdate();
					}

					leftMotor.forward();
					rightMotor.forward();
					wait.msDelay(1000);
				} else if (line1 > LLightSensor && line2 <= RLightSensor)// Robot
																			// is
																			// not
																			// straight,
																			// it
																			// is
																			// a
																			// bit
																			// to
																			// the
																			// left
				{
					Sound.beep();
					rightMotor.stop();
					thetarecord = odometer.getTheta();
					leftMotor.backward();
					wait.msDelay(1000);
					leftMotor.forward();
					Antioverturn1 = odometer.getTheta();
					while ((line1 > LLightSensor) && (!Overturn)) {
						line1 = light1.getNormalizedLightValue();
						if (Math.abs(odometer.getTheta() - Antioverturn1) >= Overturnangle) {
							Overturn = true;
						}
					}
					leftMotor.stop();

					if (Overturn) {
						leftMotor.rotate(-(45 + Overturnangle));
						Overturn = false;
						odometer.setTheta(thetarecord);
					} else {
						LSupdate();
					}

					leftMotor.forward();
					rightMotor.forward();
					wait.msDelay(1000);
				} else if (line1 <= LLightSensor && line2 <= RLightSensor)// Robot
																			// is
																			// straight,
																			// both
																			// light
																			// sensors
																			// detected
																			// the
																			// line
																			// at
																			// the
																			// same
																			// time
				{
					Sound.beep();
					stopMotor();
					
					LSupdate();

					wait.msDelay(1000);

					leftMotor.forward();
					rightMotor.forward();
				}

			}

			if (UCactive) {
				if (Math.sqrt(((goalx1 - odometer.getX())* (goalx1 - odometer.getX()) + (goaly1 - odometer.getY()) * (goaly1 - odometer.getY()))) <= 20) {
					UCactive1 = false;
				} else {
					UCactive1 = true;
				}
			}

			if (UCactive == true && UCactive1 == true) {

				distance1 = getFilteredData1();
				distance2 = getFilteredData2();

				if (cornerId == 1 && Odometer.getX()>152.4)
				{Dontturnback=true;}
				if (cornerId == 2 && Odometer.getX()<152.4)
				{Dontturnback=true;}
				if (cornerId == 3 && Odometer.getX()<152.4)
				{Dontturnback=true;}
				if (cornerId == 4 && Odometer.getX()>152.4)
				{Dontturnback=true;}
				
				
				
				if (distance1 < USdistance) {
					filter1++;
					// make sure the obstacle is real
				}

				if (distance2 < USdistance) {
					filter2++;
				}

				if (distance1 > 80) {
					filter1clean++;
					// make sure the obstacle is real
				}

				if (distance2 > 80) {
					filter2clean++;
				}

				if (filter1clean >= FILTER_CLEAN) {
					filter1 = 0;
					filter1clean = 0;
				}
				if (filter2clean >= FILTER_CLEAN) {
					filter2 = 0;
					filter2clean = 0;
				}

				if (filter1 >= FILTER_OUT || filter2 >= FILTER_OUT) {
					count1 = 1;
					stopMotor();

					goBack(-10);

					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
					}

					// goForward(10);

					// try { Thread.sleep(300); } catch (InterruptedException e)
					// {}

					// distance1 = getFilteredData1();
					// distance2 = getFilteredData2();

					// if((distance1 > USdistance) && (distance2 > USdistance) )
					// {
					// count = 1;
					// Sound.beep();
					// }

					if ((filter1 >= FILTER_OUT) && (filter2 >= FILTER_OUT)
							&& (count == 0))// Case 1: Both US sensors see
											// something
					{
						avoidOB(FacingToX);
						count = 1;
					}

					if (filter1 >= FILTER_OUT && (filter2 < FILTER_OUT)
							&& count == 0)// Case 2: Only left US sensor has
											// detected something
					{
						avoidOB(FacingToX);
						count = 1;
					}

					if ((filter1 < FILTER_OUT) && filter2 >= FILTER_OUT
							&& count == 0)// Case 3: Only right US sensor has
											// detected something
					{
						avoidOB(FacingToX);
						count = 1;
					}
				}
			}

			// if reaches the point, stop it. the destination is a circle with
			// 0.5cm radius(error)
			if (((Math.abs(goalx1 - x2) < 13) && (Math.abs(goaly1 - y2) < 13) && (count == 0))) {
				stopMotor();

				goForward(Math.abs(goaly1 - y2));
				count = 1;
				jump = false;
			}
		}

		if ((count == 1) && (count1 == 1)) {
			jump = true;

		}

		Sound.beep();

		if (jump != true) {

			/*
			 * if(count1 == 1)//US Sensor(s) have seen an obstacle {
			 * travelTo(destX, destY, true); }
			 */
			// ////////////////////////////////////////////////////////////////
			// TRAVEL TO X2Y2
			// initialize variables and get starting point from odometer
			count = 0;
			count1 = 0;
			filter1 = 0;
			filter2 = 0;
			Overturn = false;
			Antioverturn1 = 0;
			filter1clean = 0;
			filter2clean = 0;

			x0 = Odometer.getX();
			y0 = Odometer.getY();
			thetaOdo = Odometer.getTheta();
			// calculate the angle we need to turn
			X = goalx2 - goalx1;
			Y = goaly2 - goaly1;
			theta1 = Math.atan2(X, Y) / (2 * Math.PI) * 360;

			// turnTo that direction
			turnTo(theta1);

			// Moving to destination
			Motor.A.setSpeed(SLOW);
			Motor.B.setSpeed(SLOW);
			
			if(roleId == 2) {
				Motor.A.setSpeed(FAST);
				Motor.B.setSpeed(FAST);
			}

			Motor.A.forward();
			Motor.B.forward();

			drivingToY = true;
			THETA = odometer.getTheta();

			if (((THETA < 7) && (THETA > -7))
					|| ((THETA < 187) && (THETA > 173))
					|| ((THETA < -173) && (THETA > -187))) {
				FacingToX = false;
			} else if (((THETA < 97) && (THETA > 83))
					|| ((THETA < -83) && (THETA > -97))) {
				FacingToX = true;
			}

			while ((count == 0)) {
				x2 = Odometer.getX();
				y2 = Odometer.getY();
				theta2 = Odometer.getTheta();

				if (LSactive) {
					line1 = light1.getNormalizedLightValue();
					line2 = light2.getNormalizedLightValue();

					if (line1 <= LLightSensor && line2 > RLightSensor)// Robot is not straight, it
													// is a bit to the right
					{
						Sound.beep();
						leftMotor.stop();
						thetarecord = odometer.getTheta();
						rightMotor.backward();
						wait.msDelay(1000);
						rightMotor.forward();
						Antioverturn1 = odometer.getTheta();
						while ((line2 > RLightSensor) && (!Overturn)) {
							line2 = light2.getNormalizedLightValue();
							if (Math.abs(odometer.getTheta() - Antioverturn1) >= Overturnangle) {
								Overturn = true;
							}
						}
						rightMotor.stop();

						if (Overturn) {
							rightMotor.rotate(-(45 + Overturnangle));
							Overturn = false;
							odometer.setTheta(thetarecord);
						} else {
							LSupdate();
						}

						leftMotor.forward();
						rightMotor.forward();
						wait.msDelay(1000);
						Sound.playTone(900, 150);
					} else if (line1 > LLightSensor && line2 <= RLightSensor)// Robot is not
															// straight, it is a
															// bit to the left
					{
						Sound.beep();
						rightMotor.stop();
						thetarecord = odometer.getTheta();
						leftMotor.backward();
						wait.msDelay(1000);
						leftMotor.forward();
						Antioverturn1 = odometer.getTheta();
						while ((line1 > LLightSensor) && (!Overturn)) {
							line1 = light1.getNormalizedLightValue();
							if (Math.abs(odometer.getTheta() - Antioverturn1) >= Overturnangle) {
								Overturn = true;
							}
						}
						leftMotor.stop();

						if (Overturn) {
							leftMotor.rotate(-(45 + Overturnangle));
							Overturn = false;
							odometer.setTheta(thetarecord);
						} else {
							LSupdate();
						}
						wait.msDelay(1000);
						leftMotor.forward();
						rightMotor.forward();

						Sound.playTone(900, 150);
					} else if (line1 <= LLightSensor && line2 <= RLightSensor)// Robot is
															// straight, both
															// light sensors
															// detected the line
															// at the same time
					{
						Sound.beep();
						stopMotor();
						
						LSupdate();
						wait.msDelay(1000);

						leftMotor.forward();
						rightMotor.forward();

						Sound.playTone(900, 150);
					}

				}

				if (UCactive) {
					if (Math.sqrt(((goalx1 - odometer.getX())
							* (goalx1 - odometer.getX()) + (goaly1 - odometer.getY()) * (goaly1 - odometer.getY()))) <= 20) {
						UCactive1 = false;
					} else {
						UCactive1 = true;
					}
				}

				if (UCactive == true && UCactive1 == true) {

					distance1 = getFilteredData1();
					distance2 = getFilteredData2();

					if (distance1 < USdistance) {
						filter1++;
						// make sure the obstacle is real
					}

					if (distance2 < USdistance) {
						filter2++;
					}

					if (distance1 > 80) {
						filter1clean++;
						// make sure the obstacle is real
					}

					if (distance2 > 80) {
						filter2clean++;
					}

					if (filter1clean >= FILTER_CLEAN) {
						filter1 = 0;
						filter1clean = 0;
					}
					if (filter2clean >= FILTER_CLEAN) {
						filter2 = 0;
						filter2clean = 0;
					}

					if (filter1 >= FILTER_OUT || filter2 >= FILTER_OUT) {
						count1 = 1;
						stopMotor();

						goBack(-10);

						try {
							Thread.sleep(100);
						} catch (InterruptedException e) {
						}

						// goForward(10);

						// try { Thread.sleep(300); } catch
						// (InterruptedException e) {}

						// distance1 = getFilteredData1();
						// distance2 = getFilteredData2();

						// if((distance1 > USdistance) && (distance2 >
						// USdistance))
						// {
						// count = 1;
						// Sound.beep();
						// }

						if ((filter1 >= FILTER_OUT) && (filter2 >= FILTER_OUT)
								&& (count == 0))// Case 1: Both US sensors see
												// something
						{
							avoidOB(FacingToX);
							count = 1;
						}

						if (filter1 >= FILTER_OUT && (filter2 < FILTER_OUT)
								&& count == 0)// Case 2: Only left US sensor has
												// detected something
						{
							avoidOB(FacingToX);
							count = 1;
						}

						if ((filter1 < FILTER_OUT) && filter2 >= FILTER_OUT
								&& count == 0)// Case 3: Only right US sensor
												// has detected something
						{
							avoidOB(FacingToX);
							count = 1;
						}
					}
				}

				if ((Math.abs(goalx2 - x2) < 9) && (Math.abs(goaly2 - y2) < 9)
						&& (count == 0)) {
					stopMotor();

					goForward(Math.abs(goalx2 - x2));
					count = 1;

					jump = false;
				}
			}

			if ((count == 1) && (count1 == 1)) {
				jump = true;// if we detect something, we need reset travelTo
							// and start from a new point.

			}

			Sound.beep();

		}// the end of if jump

	}

	/**
	 * Turns the robot to face a specific angle
	 * 
	 * @param angle
	 *            The direction you want to robot to face (in degrees)
	 */

	public void turnTo(double angle) {

		double[] pos;
		pos = new double[3];
		odometer.getPosition(pos);

		deltaTheta = (angle) - (pos[2]);

		// LCD.drawString("" + deltaTheta, 0, 1);
		// LCD.drawString("" + pos[2], 0, 2);

		// determine which direction to turn
		if (deltaTheta < -180) {
			deltaTheta += 360;
		} else if (deltaTheta > 180) {
			deltaTheta -= 360;
		}

		Motor.A.setSpeed(SLOWEST);
		Motor.B.setSpeed(SLOWEST);
		
		if(roleId == 2) {
			Motor.A.setSpeed(SLOWER);
			Motor.B.setSpeed(SLOWER);
		}
		Motor.A.rotate(convertAngle(WHEEL_RADIUS, WIDTH, deltaTheta), true);
		Motor.B.rotate(-convertAngle(WHEEL_RADIUS, WIDTH, deltaTheta), false);

	}

	/**
	 * When call travelTo method, we define out route as go to Y position first
	 * and then finish X.
	 * 
	 * @param goalx
	 *            The X destination you want to travel to (in cm)
	 * @param goaly
	 *            The Y destination you want to travel to (in cm)
	 */
	public void defineRoute(double goalx, double goaly) {
		double x, y;

		goalx1 = odometer.getX();
		goaly1 = goaly;

		goalx2 = goalx;
		goaly2 = goaly;
	}

	public void defineRouteY(double goalx, double goaly) {
		double x, y;

		goalx1 = goalx;
		goaly1 = odometer.getY();

		goalx2 = goalx;
		goaly2 = goaly;
	}

	/**
	 * Goes straight forward a specified distance
	 * 
	 * @param distance
	 *            The distance you want to travel forward (in cm)
	 */
	public void goForward(double distance) {

		Motor.A.setSpeed(SLOW);
		Motor.B.setSpeed(SLOW);
		
		if(roleId == 2){
			Motor.A.setSpeed(FAST);
			Motor.B.setSpeed(FAST);
		}
		Motor.A.rotate(convertDistance(WHEEL_RADIUS, distance), true);
		Motor.B.rotate(convertDistance(WHEEL_RADIUS, distance), false);
	}
        
	    //grid-snapping:Update odometer
	public void LSupdate() {
		double newValue, value;
		double position = 0;
		boolean FacingX = false;

		if (((odometer.getTheta() < 7) && (odometer.getTheta() > -7))
				|| ((odometer.getTheta() < 187) && (odometer.getTheta() > 173))
				|| ((odometer.getTheta() < -173) && (odometer.getTheta() > -187))) {
			FacingX = false;
		} else if (((odometer.getTheta() < 97) && (odometer.getTheta() > 83))
				|| ((odometer.getTheta() < -83) && (odometer.getTheta() > -97))) {
			FacingX = true;
		}

		if ((odometer.getTheta() < 7) && (odometer.getTheta() > -7)) {
			position = odometer.getY() - 12.5;
		} else if (((odometer.getTheta() < 187) && (odometer.getTheta() > 173))
				|| ((odometer.getTheta() < -173) && (odometer.getTheta() > -187))) {
			position = odometer.getY() + 12.5;
		} else if ((odometer.getTheta() < 97) && (odometer.getTheta() > 83)) {
			position = odometer.getX() - 12.5;
		} else if (((odometer.getTheta() < -83) && (odometer.getTheta() > -97))) {
			position = odometer.getX() + 12.5;
		}

		value = position % 30.48;

		if (value > 15.24) {
			newValue = (position - value) + 30.48;
		} else {
			newValue = position - value;
		}

		if ((odometer.getTheta() < 7) && (odometer.getTheta() > -7)) {
			odometer.setY(newValue + 12.5);

			odometer.setTheta(0);
			turnTo(-2);
			odometer.setTheta(0);
			wait.msDelay(300);
		}

		else if ((odometer.getTheta() < 97) && (odometer.getTheta() > 83)) {
			odometer.setX(newValue + 12.5);
			odometer.setTheta(90);
			turnTo(90 - 2);
			odometer.setTheta(90);
			wait.msDelay(300);
		}

		else if ((odometer.getTheta() < -83) && (odometer.getTheta() > -97)) {
			odometer.setX(newValue - 12.5);
			odometer.setTheta(-90);
			turnTo(-90 - 2);
			odometer.setTheta(-90);
			wait.msDelay(300);
		}

		else if (((odometer.getTheta() < 187) && (odometer.getTheta() > 173))
				|| ((odometer.getTheta() < -173) && (odometer.getTheta() > -187))) {
			odometer.setY(newValue - 12.5);
			odometer.setTheta(180);
			turnTo(180 - 2);
			odometer.setTheta(180);
			wait.msDelay(300);
		}

	}

	/**
	 * Goes backward a specified distance
	 * 
	 * @param distance
	 *            The distance you want to travel backward (in cm)
	 */

	public void goBack(double distance) {

		Motor.A.setSpeed(SLOW);
		Motor.B.setSpeed(SLOW);
		Motor.A.rotate(convertDistance(WHEEL_RADIUS, distance), true);
		Motor.B.rotate(convertDistance(WHEEL_RADIUS, distance), false);
	}

	/**
	 * Starts the motors rotating at a specific speed
	 * 
	 * This method returns immediately, and the robot will keep rotating until
	 * it is stopped by something else in the code.
	 * 
	 * @param leftspeed
	 *            Speed of left motor (in tachos/sec)
	 * @param rightspeed
	 *            Speed of right motor (in tachos/sec)
	 */
	public void rotate(int leftspeed, int rightspeed) {

		Motor.A.setAcceleration(2000);
		Motor.B.setAcceleration(2000);
		Motor.A.setSpeed(Math.abs(leftspeed));
		Motor.B.setSpeed(Math.abs(rightspeed));
		if (leftspeed < 0)
			Motor.A.backward();
		else
			Motor.A.forward();
		if (rightspeed < 0)
			Motor.B.backward();
		else
			Motor.B.forward();
	}

	/**
	 * Stops both motors immediately
	 */
	public void stopMotor() {

		Motor.A.setSpeed(0);
		Motor.B.setSpeed(0);
		try {	Thread.sleep(100);	} catch (InterruptedException e) {}
	}

	/**
	 * Reads data from the ultrasonic sensor and filters it to prevent false 255
	 * values
	 * 
	 * @return The filtered data
	 */
	private int getFilteredData1() {
		int distance1;

		// do a ping
		us1.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(30);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance1 = us1.getDistance();

		if (distance1 > 255) {
			distance1 = 255;
		}

		return distance1;
	}

	/**
	 * Reads data from the ultrasonic sensor and filters it to prevent false 255
	 * values
	 * 
	 * @return The filtered data
	 */
	private int getFilteredData2() {
		int distance2;

		// do a ping
		us2.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(30);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance2 = us2.getDistance();

		if (distance2 > 255) {
			distance2 = 255;
		}

		return distance2;
	}

	/**
	 * Calculates the tachos a wheel needs to turn in order to reach a specific
	 * distance
	 * 
	 * @param radius
	 *            The radius of the robot wheel
	 * @param distance
	 *            The distance you wish to convert into tachos
	 * @return The corresponding amount of tachos that a wheel must turn in
	 *         order to travel (distance) cm
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Calculates the tachos a wheel needs to turn in order for the robot to
	 * turn a specific angle
	 * 
	 * @param radius
	 *            The radius of the robot wheel
	 * @param width
	 *            The distance between the wheels of the robot
	 * @param angle
	 *            The angle you wish to convert into tachos
	 * @return The amount of tachos to turn in order for the robot to turn a
	 *         specific angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public void avoidOB(boolean FacingX) {
		  boolean doublecheck;
		if(!FacingX)
		   {
		   if( odometer.getX() <= 30.48*5)//Started in corner 1 or 3, is on leftside
		   {
			   if(      (  (cornerId==2)||(cornerId==3)  ) && Dontturnback  )
			   {
					turnTo(-90); 
			   }
			   else{
			turnTo(90);
			   }
			
			doublecheck=checkOB();
			if(!doublecheck){
			goForward(BRICKWIDTH);
			}
			else
			{
				turnTo(-90);
				goForward(BRICKWIDTH*2);
			}

		   }
		   else if( odometer.getX() > 30.48*5)//Started in corner 1 or 3, is on rightside
		   {
			   if(      (  (cornerId==4)||(cornerId==1)  ) && Dontturnback  )
			   {
					turnTo(90); 
			   }
			   else{
			turnTo(-90);
			   }
			doublecheck=checkOB();
			if(!doublecheck){
			goForward(BRICKWIDTH);
			}
			else
			{
				turnTo(90);
				goForward(BRICKWIDTH*2);
			}

		   }
		   }
		   
		   else
		   {
		   if( odometer.getY() <= 30.48*5)//Started in corner 1 or 3, is on leftside
		   {
		    turnTo(0);

			doublecheck=checkOB();
			if(!doublecheck){
			goForward(BRICKWIDTH);
			}
			else
			{
				turnTo(180);
				goForward(BRICKWIDTH*2);
			}
		   }
		   else if(odometer.getY() > 30.48*5)//Started in corner 1 or 3, is on rightside
		   {
			turnTo(180);
			doublecheck=checkOB();
			if(!doublecheck){
			goForward(BRICKWIDTH);
			}
			else
			{
				turnTo(0);
				goForward(BRICKWIDTH*2);
			}
		   }
		   
		   }
}
    //when see a obstacle and turn to another direction,check if there is obstacle in this direction
	public boolean checkOB()
	{   
		int count=0;
		int filter1=0,filter2=0;
		while(count<=12)
		{
		distance1 = getFilteredData1();
		distance2 = getFilteredData2();
		
		 if(distance1 < 60)
		 {	
		  filter1++;
			//make sure the obstacle is real
		 }

		 if(distance2 < 60)
		 {
		  filter2++;
		 }
		 count++;
		}
		
		if((filter1>=6)||(filter2>=6))
		{
			return true;
		}
		else{return false;}
	}
	
	public void getToField(int cornerId) {
		
		int fieldDistance = 0;
		if(roleId == 1) { fieldDistance = 69; }
		else if(roleId == 2) { fieldDistance = 65; }
		
		
		if (cornerId == 1) {
			turnTo(FIELD_ANGLE);

			// go forward 30*cos(31 deg) to get to the next tile
			goForward(fieldDistance);

			// turn toward x-axis
			if(roleId == 1) { turnTo(90); }
			else { turnTo(0); }
			
		}

		else if (cornerId == 2) {
			turnTo(-FIELD_ANGLE);

			// go forward 30*cos(31 deg) to get to the next tile
			goForward(fieldDistance);

			if(roleId == 1) { turnTo(270); }
			else { turnTo(0); }
		}

		else if (cornerId == 3) {
			turnTo(-180 + FIELD_ANGLE);

			// go forward 30*cos(31 deg) to get to the next tile
			goForward(fieldDistance);

			if(roleId == 1) { turnTo(270); }
			else { turnTo(180); }
		}

		else if (cornerId == 4) {
			turnTo(180 - FIELD_ANGLE);

			// go forward 30*cos(31 deg) to get to the next tile
			goForward(fieldDistance);

			if(roleId == 1) { turnTo(90); }
			else { turnTo(180); }
		}

		// no parameter specified, default is corner 1
		else {
			Sound.buzz();
		}
		
		boolean forget = false;
		if(forget) {
			rotate(75, 75);
			sawLine1 = 0;
			sawLine2 = 0;
			count = 0;
	
			while (loop) {
	
				line1 = light1.getNormalizedLightValue();
				line2 = light2.getNormalizedLightValue();
	
				if (line2 < RLightSensor && line1 < LLightSensor
						&& (sawLine2 < 1 || sawLine1 < 1)) {
					count = 2;
					Sound.beep();
					sawLine2++;
					sawLine1++;
					stopMotor();
				}
				if (line1 < LLightSensor && sawLine1 < 1) {
					++count;
					Sound.beep();
					sawLine1++;
					Motor.A.stop();
				}
	
				if (line2 < RLightSensor && sawLine2 < 1) {
					++count;
					Sound.beep();
					sawLine2++;
					Motor.B.stop();
				}
	
				if (count == 2) {
	
					goBack(-12.5);
					loop = false;
					try {
						Thread.sleep(300);
					} catch (InterruptedException e) {
					}
				}
	
			}
	
			if (cornerId == 1 || cornerId == 2) {
				turnTo(0);
			} else if (cornerId == 3 || cornerId == 4) {
				turnTo(180);
			} else {
				turnTo(0);
			}
	
			loop = true;
			rotate(75, 75);
			sawLine1 = 0;
			sawLine2 = 0;
			count = 0;
	
			while (loop) {
	
				line1 = light1.getNormalizedLightValue();
				line2 = light2.getNormalizedLightValue();
	
				if (line2 < RLightSensor && line1 < LLightSensor
						&& (sawLine2 < 1 || sawLine1 < 1)) {
					count = 2;
					Sound.beep();
					sawLine2++;
					sawLine1++;
					stopMotor();
				}
				if (line1 < LLightSensor && sawLine1 < 1) {
					++count;
					Sound.beep();
					sawLine1++;
					Motor.A.stop();
				}
	
				if (line2 < RLightSensor && sawLine2 < 1) {
					++count;
					Sound.beep();
					sawLine2++;
					Motor.B.stop();
				}
	
				if (count == 2) {
					goBack(-12.5);
					loop = false;
					try {
						Thread.sleep(200);
					} catch (InterruptedException e) {
					}
				}
			}
		

			boolean[] updateAll = new boolean[] { true, true, true };
	
			if (cornerId == 1) {
				odometer.setPosition(new double[] { 60.96, 30.48, 0 }, updateAll);
			}
	
			else if (cornerId == 2) {
				odometer.setPosition(new double[] { 243.84, 30.48, 0 }, updateAll);
			}
	
			else if (cornerId == 3) {
				odometer.setPosition(new double[] { 243.84, 274.32, 180 },updateAll);
			}
	
			else if (cornerId == 4) {
				odometer.setPosition(new double[] { 60.96, 274.32, 180 }, updateAll);
			}
	
			// no parameter specified
			else {
				Sound.buzz();
			}
		}

		Sound.playTone(1000, 150);
		try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
		}

	}
	


}