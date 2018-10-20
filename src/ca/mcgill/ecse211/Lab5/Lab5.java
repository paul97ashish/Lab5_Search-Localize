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
	public static final double TRACK = 15.7;
	public static Display odometryDisplay;
	public static Navigation navigation;
	public static ObstacleDetect obstacleDetect;
	public static Localization localization;
	public static EV3GyroSensor gyro;
	private static final double TILE_SIZE=30.48;
	private static int UUX=6;
	private static int UUY=6;
	private static int LLX=2;
	private static int LLY=2;
	private static int SC=0;
	private static int TR=2;
	private static final double[][] COORDONATES = { { 1, 1, 0 }, { 7 , 1, 270 },
			{ 7 , 7 , 180 }, { 1, 7 , 90 } };
	public static Odometer odometer;
	public static float[] data;
	public static SampleProvider gyroAngle;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation

		// implementation
		odometryDisplay = new Display(lcd); // No need to change
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		navigation = new Navigation(); //create an instance of the navigation class
		float[] usData = new float[usDistance.sampleSize()]; //create a sample array
		UltrasonicPoller usPoller = null;
		gyro = new EV3GyroSensor(LocalEV3.get().getPort("S3"));
		gyroAngle = gyro.getAngleMode();
		data = new float[gyroAngle.sampleSize()];



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
			localization = new Localization(true,gyro);
			localization.run();
		}else {
			localization = new Localization(false,gyro); // else run the rising edge one
			localization.run();
		}
		navigation.gyro = gyro;
		navigation.useGyro = true;
		navigation.angle = data;
		navigation.offset =(int) COORDONATES[SC][2];

		odometer.setXYT(COORDONATES[SC][0]*TILE_SIZE, COORDONATES[SC][1]*TILE_SIZE, COORDONATES[SC][2]);
		navigation.travelTo(COORDONATES[SC][0], LLY);
		navigation.travelTo(LLX, LLY);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
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
