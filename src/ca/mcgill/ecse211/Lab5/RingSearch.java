package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
/**
 * This method checks if there's a ring in the search area and detects in that case
 * @author Zakaria Essadaoui
 * @author Carl ElKhoury
 *
 */
public class RingSearch {
	// creating motor that moves the sensor
	private static final EV3MediumRegulatedMotor SensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor leftMotor = Lab5.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Lab5.getRightMotor();
	// initiating needed fields
	public ColorDetection detect;
	private static int distance;

	private static UltrasonicPoller usPoller;
	private static double LastX, LastY;
	private static Navigation navigation;
	private static final int SPEED = 50;
	int ringValue = 5;
	private static double radius = Lab5.WHEEL_RAD;
	boolean notFound = true;
	/**
	 * this our constructor
	 * @param poller
	 * @param navig
	 */
	public RingSearch(UltrasonicPoller poller, Navigation navig) {
		// System.out.println("RingSerch created");
		
		// we turn the sensor to face the search area
		SensorMotor.setSpeed(100);
		SensorMotor.rotate(-120);
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
		// checks if a ring is detected
		distance = usPoller.getDistance();
		// System.out.println(distance);
		if (distance < Lab5.rangeDetection && checkRing()) {
			
			//Lab5.localization.linedetect();
			
			// move until the minimum distance is detected
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.forward();
			rightMotor.forward();
			int newdistance = usPoller.getDistance();
			// move until the distance detected increases
			while ((newdistance - distance) <= 0) {
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				distance = newdistance;
				newdistance = usPoller.getDistance();
			}
			// saving the target x and target y 
			double TargX = navigation.x;
			double TargY = navigation.y;
			System.out.println("rechecked distance :" + newdistance);
			// going to the next line and perform a localization
			Lab5.localization.linedetect();
			turnBy(90);
			move(-7);
			Lab5.localization.linedetect();
			// save our position
			LastX = Lab5.odometer.getXYT()[0];
			LastY = Lab5.odometer.getXYT()[1];
		
			// getting closer to the sensor and slowing down the motors
			move(distance-20);
			
			leftMotor.setSpeed(SPEED);
			rightMotor.setSpeed(SPEED);
			ringValue = detect.detect();
			// keeps checking for a ring color until it find a known ring
			// Or if the distance crossed is bigger than the range detection(In case of false positives)
			while (ringValue == 5 && Math.abs(LastX- Lab5.odometer.getXYT()[0])<Lab5.rangeDetection &&
					Math.abs(LastY- Lab5.odometer.getXYT()[1])<Lab5.rangeDetection) {
				leftMotor.forward();
				rightMotor.forward();
				try {
					Thread.sleep(40);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				ringValue = detect.detect();
			}
			/*If the ring was found, beep and go the Upper Right
			 * other wise, beep twice and go back to the last X and Y
			 */
			if (ringValue == (Lab5.TR - 1)) {
				Sound.beep();
				navigation.travelTo(Lab5.UUX, Lab5.UUY);
				notFound = false;
			} else {
				Sound.beep();
				Sound.beep();
			}
			// going back the LastX and Last Y position
			if(notFound) {
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
			double theta=Lab5.odometer.getXYT()[2];
			// travelling close to the next tile without checking to not detect the same ring
			if (theta>225 && theta< 315) { // goind on the first side
				navigation.travelTo((LastX / Lab5.TILE_SIZE), (LastY / Lab5.TILE_SIZE)+0.7);
			}else if(theta<45 || theta> 315) {// second side
				navigation.travelTo((LastX / Lab5.TILE_SIZE)+0.7, (LastY / Lab5.TILE_SIZE));
			}else if(theta > 45 && theta < 135) {// third side
				navigation.travelTo((LastX / Lab5.TILE_SIZE), (LastY / Lab5.TILE_SIZE)-0.7);
			}else if(theta >135 && theta< 225) {// fourth side
				navigation.travelTo((LastX / Lab5.TILE_SIZE)-0.7, (LastY / Lab5.TILE_SIZE));
			}
			// travelling to our target again
			navigation.travelTo(TargX, TargY, notFound);
			}

		}

	}
	/**
	 * This method makes the robot move by the distance that was passed to it
	 * @param distance to move
	 * @return void
	 */
	private void move(double distance) {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(convertDistance(radius, distance), true);
		rightMotor.rotate(convertDistance(radius, distance), false);
	}
	/**
	 * This method makes the robot turn (clockwise) by the angle that was passed to it
	 * @param theta to turn
	 * @return void
	 */
	private void turnBy(double theta) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(convertAngle(radius, Lab5.TRACK, theta), true); // turns to the origin point
		rightMotor.rotate(-convertAngle(radius, Lab5.TRACK, theta), false);
	}
	/**
	 * This method makes sure that a ring is there
	 * @return boolean
	 */
	public boolean checkRing() {
		int distance1 = usPoller.getDistance();
		int distance2 = usPoller.getDistance();
		int distance3 = usPoller.getDistance();
		return (distance1 < Lab5.rangeDetection) && (distance2 < Lab5.rangeDetection) && (distance3 < Lab5.rangeDetection);
	}
	/**
	 * This method returns the number of the ring detected 
	 * @return ring number
	 */
	public int ringValue() {
		return ringValue + 1;
	}
	/**
	 * This method calculate the angle that must be passed to the motor 
	 * using the radius of the wheel and the distance we want the robot to cross
	 * @param radius
	 * @param distance
	 * @return	corresponding distance
	 */
	private static int convertDistance(double radius, double distance) { // converts distance to wheel rotations
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * This method calculate the angle that the motor must turn 
	 * in order for the robot to turn by an certain angle
	 * @param radius
	 * @param width
	 * @param angle
	 * @return Angle by which the wheels must turn
	 */
	private static int convertAngle(double radius, double width, double angle) { // converts angle to radians for degree
																					// rotation
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
