package ca.mcgill.ecse211.Lab5;


import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.Lab5.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
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
	public static final double TRACK = 15.7;
	public static Display odometryDisplay;
	public static Navigation navigation;
	public static ObstacleDetect obstacleDetect;
	public static Localization localization;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation

		// implementation
		odometryDisplay = new Display(lcd); // No need to change
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		navigation = new Navigation(); //create an instance of the navigation class
		float[] usData = new float[usDistance.sampleSize()]; //create a sample array
		UltrasonicPoller usPoller = null;



		do {
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
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			obstacleDetect = new ObstacleDetect();//start the obstacle detect class that sample the ultrasonic sensor
	    	usPoller = new UltrasonicPoller(usDistance, usData, obstacleDetect );
	        usPoller.start();

			// Start correction if right button was pressed
			 if (buttonChoice == Button.ID_LEFT) { // if the user selects left run falling edge localization
				localization = new Localization(true);
				localization.start();
			 }else {
				 localization = new Localization(false); // else run the rising edge one
				 localization.start();
			 }
			 

		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		do {
			localization.nextStep=true; // pressing on the escape button once will trigger this statement which runs the next step of the localization: le light sensor localization
		}while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	
		
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
