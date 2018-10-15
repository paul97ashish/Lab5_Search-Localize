package ca.mcgill.ecse211.attempt;


import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This our main class. 
 * We use it to ask the user what kind of Localization he wan't to chose 
 * and also start the odometer thread.
 * @author Group 21
 *
 */
public class Lab5 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.6;
	public static final int [][]SEARCH_AREA= {{7,7},{3,3}};
	public static final int SC=2;
	public static final double TILE_SIZE=30.48;
	public static final double[][] COORDONATES= {{0,0,0},{6*TILE_SIZE, 0, 270}, {6*TILE_SIZE, 6*TILE_SIZE, 180},{0, 6*TILE_SIZE, 90}};
	public static Odometer odometer;
	
	
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		int buttonChoice;

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		// create the display
		
		// ask the user to chose the localization type and wait for the input

			// clear the display
			lcd.clear();

		      lcd.drawString("Wainting 			", 0, 0);
		      lcd.drawString("for 				", 0, 1);
		      lcd.drawString("button			", 0, 2);
		      lcd.drawString("Press				", 0, 3);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		
		// create odometer thread
			lcd.clear();
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		// create an Ultrasonic Localizer
		UltrasonicLocalizer ultra=new UltrasonicLocalizer(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
		lcd.clear();
		//Run falling edge localization
		ultra.FallingEdge();
		
		odometer.setTheta(0);
		// start light localization
		LightLocalizer light=new LightLocalizer(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
		light.Localize();
		odometer.setXYT(COORDONATES[SC][0], COORDONATES[SC][1], COORDONATES[SC][2]);
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}

