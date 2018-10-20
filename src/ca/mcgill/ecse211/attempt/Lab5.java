package ca.mcgill.ecse211.attempt;

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This our main class. We use it to ask the user what kind of Localization he
 * wan't to chose and also start the odometer thread.
 * 
 * @author Group 21
 *
 */
public class Lab5 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.7;
	public static final int[][] SEARCH_AREA = { { 6, 6 }, { 2, 2 } };
	public static final int SC = 0;
	public static final int TR = 3;
	public static final double TILE_SIZE = 31.48;
	public static final double[][] COORDONATES = { { TILE_SIZE, TILE_SIZE, 0 }, { 7 * TILE_SIZE, TILE_SIZE, 270 },
			{ 7 * TILE_SIZE, 7 * TILE_SIZE, 180 }, { TILE_SIZE, 7 * TILE_SIZE, 90 } };
	public static Odometer odometer;
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance

	private static EV3GyroSensor gyro=new EV3GyroSensor(LocalEV3.get().getPort("S3"));
	private static SampleProvider gyroAngle=gyro.getAngleMode();
	private static float[]	Angle=new float[gyroAngle.sampleSize()];
	private static float[] data;
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {


		int buttonChoice;

		// Odometer related objects
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD, gyroAngle); // TODO Complete implementation
		// create the display

		// ask the user to chose the localization type and wait for the input

		// clear the display
		lcd.clear();

		lcd.drawString("Wainting 			", 0, 0);
		lcd.drawString("for 				", 0, 1);
		lcd.drawString("button			", 0, 2);
		lcd.drawString("Press				", 0, 3);

		buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(-convertAngle(WHEEL_RAD,TRACK, 360), true);
		rightMotor.rotate(+convertAngle(WHEEL_RAD, TRACK, 360), false);
		// create odometer thread
		lcd.clear();
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		// create an Ultrasonic Localizer
		UltrasonicLocalizer ultra = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK,
				usSensor);
		lcd.clear();
		// Run falling edge localization
		ultra.FallingEdge();
		ultra = null;
		odometer.setTheta(0);
		gyro.reset();
		odometer.Last=0.0;
		// start light localization
		LightLocalizer light = new LightLocalizer(leftMotor, rightMotor, odometer, WHEEL_RAD, TRACK);
		light.Localize();
		light = null;
		odometer.setXYT(COORDONATES[SC][0], COORDONATES[SC][1], COORDONATES[SC][2]);
		gyro.reset();
		data=new float[gyro.sampleSize()];
		odometer.Last=0.0;
		odometer.setTheta(0);
		Navigation nav = new Navigation(TILE_SIZE, leftMotor, rightMotor, TRACK, WHEEL_RAD, TR, usSensor);
		// got to the lower left corner
		System.out.println(odometer.getXYT()[2]);
		nav.TravelTo(SEARCH_AREA[1][0] * TILE_SIZE, SEARCH_AREA[1][1] * TILE_SIZE, false);
		// travelling to every point in the search area looking the ring
		System.out.println(odometer.getXYT()[2]);
		boolean first = true;
		int j=SEARCH_AREA[1][1];
		for (int i = SEARCH_AREA[1][0]; i <= SEARCH_AREA[0][0]; i++) {
			gyro.fetchSample(data, 0);
			odometer.setTheta(data[0]);
			if (nav.TravelTo(i * TILE_SIZE, j * TILE_SIZE, true))
				break;
			
			if (first) {
				if (nav.TravelTo(i * TILE_SIZE, SEARCH_AREA[0][1] * TILE_SIZE, true))
					break;
				first = false;
				j=SEARCH_AREA[0][1];
			} else {
				if (nav.TravelTo(i * TILE_SIZE, SEARCH_AREA[1][1] * TILE_SIZE, true))
					break;
				first = true;
				j=SEARCH_AREA[1][1];
			}
		}
		// go to the upper left corner
		nav.TravelTo(SEARCH_AREA[0][0] * TILE_SIZE, SEARCH_AREA[0][1] * TILE_SIZE, false);
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
}
