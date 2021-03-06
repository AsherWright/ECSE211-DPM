/*
 * BlockDetector.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding Objects
 * Group 53
 * Uses the color/light sensor to detect and report on objects it sees.
 * If the object is a styrofoam block, then it reports block.
 * Otherwise, it reports NOT BLOCK
 */
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class BlockDetector extends Thread {
	//constants
		//class variables
	//these are the different block profiles. They are initialized in the constructor
	private double[] blueBlockReading;
	private double[] darkBlueBlockReading;
	//the error that R,G, B can be off for it to still consider it a certain object.
	private static final double DETECTIONTHRESHOLDERROR = 0.5;
	//color sensor variables
	private SampleProvider colorSensor;
	private float[] colorData;
	//Reading properties
	private boolean isReadingBlock;
	private String blockType;

	//constructor
	public BlockDetector(SampleProvider colorSensor, float[] colorData) {
		//get incoming values for variables
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		
		//initialize variables
		blockType = "";
		isReadingBlock = false;
		//initialize block profiles
		blueBlockReading = new double[3];
		darkBlueBlockReading = new double[3];
		blueBlockReading[0] = 0.95;
		blueBlockReading[1] = 1.4;
		blueBlockReading[2] = 1.15;
		darkBlueBlockReading[0] = 0.2;
		darkBlueBlockReading[1] = 0.5;
		darkBlueBlockReading[2] = 0.7;

		
	}
	/*
	 * (non-Javadoc)
	 * @see java.lang.Thread#run()
	 * The method that is called when this thread starts.
	 */
	public void run(){
		while(true){
			//gets the data from the color sensor.
			colorSensor.fetchSample(colorData, 0);
			
			//checks the reading and compares it to each profile.
			double[] BlueBlockError = new double[3];
			double totalNoObjectError = 0;
			double[] DarkBlueBlockError  = new double[3];
			//go through R,G,B
			for(int i = 0; i < 3; i++){
				BlueBlockError[i] = Math.abs(colorData[i]*10 - blueBlockReading[i]);
				totalNoObjectError += Math.abs(colorData[i]*10);
				DarkBlueBlockError[i] = Math.abs(colorData[i]*10 - darkBlueBlockReading[i]);
			}
			//If our reading is within the allowed number to consider it a blue block, update what it sees.
			if(totalNoObjectError < 0.25){
				blockType = "";
				isReadingBlock = false;
			}else if(BlueBlockError[0] < DETECTIONTHRESHOLDERROR && BlueBlockError[1] < DETECTIONTHRESHOLDERROR &&  BlueBlockError[2] < DETECTIONTHRESHOLDERROR ){
				blockType = "BLOCK";
				isReadingBlock =true;
			}else if(DarkBlueBlockError[0] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[1] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[2] < DETECTIONTHRESHOLDERROR){
				blockType = "BLOCK";
				isReadingBlock = true;
			}else{
				blockType = "NOT BLOCK";
				isReadingBlock = false;
			}
			//now sleep so that we don't call this too often.
			try {
				Thread.sleep(200);											// sleep for 200 mS
			} catch (Exception e) {
				System.out.println("Error: " + e.getMessage());
			}
		}
	}

	//accessors
	public float[] getColorData(){
		synchronized (this) {
			return colorData;	
		}
	}
	public String getBlockType(){
		synchronized (this) {
			return blockType;	
		}
	
	}
	public boolean isReadingBlock(){
		synchronized (this) {
			return isReadingBlock;	
		}
	}

}
