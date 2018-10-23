package ca.mcgill.ecse211.Lab5;


import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.Lab5.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Gyroscope;
import lejos.robotics.SampleProvider;

public class Lab5 {
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 14.9;

	public static Navigation navigation;
	
	public static Localization localization;
	public static EV3GyroSensor gyro;
	public static UltrasonicPoller usPoller;
	public static final double TILE_SIZE=31.48;
	private static int UUX=7;
	private static int UUY=7;
	private static int LLX=1;
	private static int LLY=1;
	private static int SC=0;
	public static int TR=2;
	private static final double[][] COORDONATES = { { 1, 1, 0 }, { 7 , 1, 270 },
			{ 7 , 7 , 180 }, { 1, 7 , 90 } };
	public static Odometer odometer;
	public static float[] data;

	public static RingSearch search;
	
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation

		// implementation
	
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		navigation = new Navigation(); //create an instance of the navigation class
		float[] usData = new float[usDistance.sampleSize()]; //create a sample array


		// clear the display
		lcd.clear();


		lcd.clear();

		lcd.drawString("        |       ", 0, 0);
		lcd.drawString("<Falling| Rising>  ", 0, 1);
		lcd.drawString("  edge  |  edge   ", 0, 2);
		lcd.drawString("        |       ", 0, 3);
		lcd.drawString("        |        ", 0, 4);
		//			navigation.turn360(true);
		buttonChoice = Button.waitForAnyPress(); 									// Record choice (left or right press)

		// Start odometer and display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		
		
		usPoller = new UltrasonicPoller(usDistance, usData );
		
		// Start correction if right button was pressed
		if (buttonChoice == Button.ID_LEFT) { // if the user selects left run falling edge localization
			localization = new Localization(true);
			localization.run();
		}else {
			localization = new Localization(false); // else run the rising edge one
			localization.run();
		}
	
		buttonChoice = Button.waitForAnyPress(); 	
		
		odometer.setXYT(COORDONATES[SC][0]*TILE_SIZE, COORDONATES[SC][1]*TILE_SIZE, COORDONATES[SC][2]);
		
		navigation.travelTo(COORDONATES[SC][0], LLY);
		navigation.travelTo(LLX, LLY);
		
	//	navigation.obstacle=true;
		search=new RingSearch(usPoller, navigation);
		
		
		boolean detected=false;
		navigation.travelTo(LLX, UUY, true);
		detected=(search.ringValue()==TR);
		localization.lightLoc();
		double[] rounded=round(odometer.getXYT()[0],odometer.getXYT()[1]);
		odometer.setXYT(rounded[0], rounded[1], 90);
		if(!detected) {
			navigation.travelTo(UUX, UUY, true);
		
		detected=(search.ringValue()==TR);
		localization.lightLoc();
		rounded=round(odometer.getXYT()[0],odometer.getXYT()[1]);
		odometer.setXYT(rounded[0], rounded[1], 180);
		}
		
		if(!detected) {navigation.travelTo(UUX, LLY, true);
		detected=(search.ringValue()==TR);
		
		localization.lightLoc();
		rounded=round(odometer.getXYT()[0],odometer.getXYT()[1]);
		odometer.setXYT(rounded[0], rounded[1], 270);
		}
		if(!detected) {navigation.travelTo(LLX, LLY, true);
		detected=(search.ringValue()==TR);
		localization.lightLoc();
		rounded=round(odometer.getXYT()[0],odometer.getXYT()[1]);
		odometer.setXYT(rounded[0], rounded[1], 0);
		}
	//	navigation.obstacle=false;
		navigation.travelTo(UUX, UUY);
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);


	}
	public static double [] round(double x, double y){
		double X=x/TILE_SIZE;
		double Y=y/TILE_SIZE;
		double errorx= X-(int)X;
		if(errorx>=0.5) {
			X=(int)X+1;
		}else {
			X=(int)X;
		}
		double errory=Y-(int)Y;
		if(errory>=0.5) {
			Y=(int)Y+1;
		}else {
			Y=(int)Y;
		}
		
		return new double[]{X*TILE_SIZE,Y*TILE_SIZE};	
		
	}


	public static EV3LargeRegulatedMotor getLeftMotor() {			//creating public methods to allow for calling of motors and key values in other classes
		return leftMotor;
	}
	public static EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	public static double getRadius() {
		return WHEEL_RAD;
	}

	public static double getTrack() {
		return TRACK;
	}
}
