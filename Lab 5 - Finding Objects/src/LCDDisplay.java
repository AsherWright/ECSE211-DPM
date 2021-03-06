/*
 * LCDDisplay.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding objects
 * Group 53
 * This class controls the text that is displayed on the LCD screen. 
 * Adjusted from LCDInfo from MyCourses.
 */
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LCDDisplay implements TimerListener{
	//class variables
	public static final int LCD_REFRESH = 100;
	//we need the detector, odometer, and US variables to display all of the information
	private BlockDetector detector;
	private Odometer odo;
	double[] position;
	private SampleProvider usSensor;
	private float[] usData;
	// variables to do with LCD + timerlistener
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	
	// constructor
	public LCDDisplay(Odometer odo, BlockDetector detector, SampleProvider usSensor, float[] usData) {
		//get incoming variables
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.detector = detector;
		position = new double[3]; //initialize array
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		// start the timer
		lcdTimer.start();
	}
	@Override
	public void timedOut() {
		LCD.clear();
		// TODO Auto-generated method stub
		odo.getPosition(position);
		String isThereBlock = "";
		String blockType = "";
		//get the type of block
		blockType = detector.getBlockType();
		//if we are reading a block, say there is an object detected
		if(detector.isReadingBlock()){
			isThereBlock = "Object Detected";
		}
		//get rgb data
		float[] RGB = detector.getColorData();
		// clear the lines for displaying the robot information
		LCD.drawString(isThereBlock, 0, 0);
		LCD.drawString(blockType, 0, 1);
		LCD.drawString(String.valueOf(formattedDoubleToString(position[0],2).toCharArray()), 3, 2);
		LCD.drawString(String.valueOf(formattedDoubleToString(position[1],2).toCharArray()), 3, 3);
		LCD.drawString(String.valueOf(formattedDoubleToString(position[2],2).toCharArray()), 3, 4);		
		LCD.drawString(formattedDoubleToString(RGB[0]*10, 2) + "," + formattedDoubleToString(RGB[1]*10, 2)+ "," + formattedDoubleToString(RGB[2]*10, 2) , 3, 5);
		LCD.drawString(formattedDoubleToString(getFilteredData(), 3), 3, 6);

	}
	
	/*
	 * formats a double to a string with a certain amount of decimal places.
	 */
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;
		
		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";
		
		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;
			
			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}
			
			result += stack;
		}
		
		// put the decimal, if needed
		if (places > 0) {
			result += ".";
		
			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long)x);
			}
		}
		
		return result;
	}
	
	/*
	 * gets the Us data from the US sensor
	 */
	private float getFilteredData() {
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
