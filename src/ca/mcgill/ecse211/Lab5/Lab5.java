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
/**
 * This is the main class
 * It is used to create all the thread and calling the appropriate methods
 * @author Zakaria Essadaoui
 * @author Carl ElKhoury
 *
 */
public class Lab5 {
	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 14.9;

	public static Navigation navigation;

	public static Localization localization;
	public static UltrasonicPoller usPoller;
	public static final double TILE_SIZE = 30.48;
	// creating ring related fields
	public static int UUX = 7;
	public static int UUY = 7;
	private static int LLX = 2;
	private static int LLY = 2;
	private static int SC = 0;
	public static int TR = 2;
	private static final double[][] COORDONATES = { { 1, 1, 0 }, { 7, 1, 270 }, { 7, 7, 180 }, { 1, 7, 90 } };
	public static Odometer odometer;
	public static float[] data;
	public static double rangeDetection;
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
		navigation = new Navigation(); // create an instance of the navigation class
		float[] usData = new float[usDistance.sampleSize()]; // create a sample array

		// clear the display
		lcd.clear();



		lcd.drawString("Waiting ", 0, 0);
		lcd.drawString("For", 0, 1);
		lcd.drawString("Any  ", 0, 2);
		lcd.drawString("Press", 0, 3);
	
		buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		// Start odometer thread
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		// create an ultrasonic poler
		usPoller = new UltrasonicPoller(usDistance, usData);
		// start our localization routine
		localization = new Localization(true);
		localization.run();
		
		// wait for a press to proceed to the next part
		buttonChoice = Button.waitForAnyPress();
		// set the odometer coordonates according to our starting corner
		odometer.setXYT(COORDONATES[SC][0] * TILE_SIZE, COORDONATES[SC][1] * TILE_SIZE, COORDONATES[SC][2]);
		// travel to the lower left corner of search area
		navigation.travelTo(COORDONATES[SC][0], LLY);
		navigation.travelTo(LLX, LLY);
		// created a an instance of Ring Search
		search = new RingSearch(usPoller, navigation);

		boolean detected = false;
		// Set our detection range for the US sensor (this will change according to the dirrection)
		rangeDetection = (UUX - LLX) * TILE_SIZE / 2;
		// start travelling to the first corner of the search area and checks for rings
		navigation.travelTo(LLX, UUY, true);
		detected = (search.ringValue() == TR);
		// perform light localization at the corner if the ring isn't detetected
		if(!detected)localization.lightLoc();
		// round its position
		double[] rounded = round(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
		odometer.setXYT(rounded[0], rounded[1], 90);
		// if the color is not detected, we search for it from every side in a similar way
		// we just need to change our range detection accordingly
		if (!detected) {
			rangeDetection = (UUY - LLY) * TILE_SIZE / 2;
			navigation.travelTo(UUX, UUY, true);

			detected = (search.ringValue() == TR);
			localization.lightLoc();
			rounded = round(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
			odometer.setXYT(rounded[0], rounded[1], 180);
		}

		if (!detected) {
			rangeDetection = (UUX - LLX) * TILE_SIZE / 2;
			navigation.travelTo(UUX, LLY, true);
			detected = (search.ringValue() == TR);

			localization.lightLoc();
			rounded = round(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
			odometer.setXYT(rounded[0], rounded[1], rounded[2]);
		}
		if (!detected) {
			rangeDetection = (UUY - LLY) * TILE_SIZE / 2;
			navigation.travelTo(LLX, LLY, true);
			detected = (search.ringValue() == TR);
			localization.lightLoc();
			rounded = round(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2]);
			odometer.setXYT(rounded[0], rounded[1], rounded[2]);
		}
		// At the end, go back the upper right corner
		if(!detected)navigation.travelTo(UUX, UUY);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}
	/**
	 * This rounding method get the values from the odometer 
	 * and return the closest grid intersection and orientation
	 * This is done after performing light localization 
	 * @param x
	 * @param y
	 * @param theta
	 * @return real coordonates of the robot
	 */
	public static double[] round(double x, double y, double theta) {
		double X = x / TILE_SIZE;
		double Y = y / TILE_SIZE;
		// check for closest X
		double errorx = X - (int) X;
		if (errorx >= 0.5) {
			X = (int) X + 1;
		} else {
			X = (int) X;
		}
		// checks for closest Y
		double errory = Y - (int) Y;
		if (errory >= 0.5) {
			Y = (int) Y + 1;
		} else {
			Y = (int) Y;
		}
		// check for closest theta
		double ret = 0;
		if (theta > 225 && theta < 315) { // goind on the first side
			theta = 270;
		} else if (theta < 45 || theta > 315) {// second side
			theta = 0;
		} else if (theta > 45 && theta < 135) {// third side
			theta = 90;
		} else if (theta > 135 && theta < 225) {// fourth side
			theta = 180;
		}
		return new double[] { X * TILE_SIZE, Y * TILE_SIZE, ret };

	}
	/**
	 * Getter method
	 * @return o
	 */
	public static EV3LargeRegulatedMotor getLeftMotor() { // creating public methods to allow for calling of motors and
															// key values in other classes
		return leftMotor;
	}
	/**
	 * Getter method
	 * @return rightMotor
	 */
	public static EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}
	/**
	 * Getter method
	 * @return radius of the wheel
	 */
	public static double getRadius() {
		return WHEEL_RAD;
	}
	/**
	 * Getter method
	 * @return the track
	 */
	public static double getTrack() {
		return TRACK;
	}
}
