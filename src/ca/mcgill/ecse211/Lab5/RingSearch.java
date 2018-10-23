package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class RingSearch {
	private static final EV3MediumRegulatedMotor SensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor leftMotor = Lab5.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Lab5.getRightMotor();
	private static ColorDetection detect;
	private static int distance;

	private static UltrasonicPoller usPoller;
	private static double LastX, LastY;
	private static Navigation navigation;
	private static final int SPEED = 50;
	int ringValue = 5;
	private static double radius = Lab5.WHEEL_RAD;
	boolean notFound = true;

	public RingSearch(UltrasonicPoller poller, Navigation navig) {
		// System.out.println("RingSerch created");
		SensorMotor.setSpeed(100);
		SensorMotor.rotate(-100);
		detect = new ColorDetection();
		usPoller = poller;
		navigation = navig;
	}

	public void look() {
		/*
		 * Last = usPoller.getDistance(); System.out.println(Last);
		 */
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		distance = usPoller.getDistance();
		// System.out.println(distance);
		if (distance < 95) {
			// Lab5.localization.linedetect();
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.forward();
			rightMotor.forward();
			int newdistance = usPoller.getDistance();
			if (newdistance > 95)
				return;
			while ((newdistance - distance) <= 0) {
				distance = newdistance;
				newdistance = usPoller.getDistance();
			}

			double TargX = navigation.x;
			double TargY = navigation.y;
			System.out.println("rechecked distance :" + newdistance);
			Lab5.localization.linedetect();
			LastX = Lab5.odometer.getXYT()[0];
			LastY = Lab5.odometer.getXYT()[1];
			turnBy(90);
			move(distance - 20);

			leftMotor.setSpeed(SPEED);
			rightMotor.setSpeed(SPEED);
			ringValue = detect.detect();
			while (ringValue == 5) {
				leftMotor.forward();
				rightMotor.forward();
				try {
					Thread.sleep(40);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				ringValue = detect.detect();
			}
			if (ringValue == (Lab5.TR - 1)) {
				Sound.beep();
				notFound = false;
			} else {
				Sound.beep();
				Sound.beep();
			}
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);
			leftMotor.backward();
			rightMotor.backward();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			leftMotor.setSpeed(1);
			rightMotor.setSpeed(1);
			leftMotor.stop(true);
			rightMotor.stop();
			navigation.travelTo(LastX / Lab5.TILE_SIZE, LastY / Lab5.TILE_SIZE);
			turnBy(90);
			move(13);
			navigation.travelTo(TargX, TargY, notFound);

		}

	}

	private void move(double distance) {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(convertDistance(radius, distance), true);
		rightMotor.rotate(convertDistance(radius, distance), false);
	}

	private void turnBy(double theta) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertAngle(radius, Lab5.TRACK, theta), true); // turns to the origin point
		rightMotor.rotate(-convertAngle(radius, Lab5.TRACK, theta), false);
	}

	public boolean checkRing() {
		int distance1 = usPoller.getDistance();
		int distance2 = usPoller.getDistance();
		int distance3 = usPoller.getDistance();
		return (distance1 < 100) && (distance2 < 100) && (distance3 < 100);
	}

	public int ringValue() {
		return ringValue + 1;
	}

	private static int convertDistance(double radius, double distance) { // converts distance to wheel rotations
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) { // converts angle to radians for degree
																					// rotation
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
