package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.odometer.OdometerData;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;

/**
 * This class is responsible for the calculations of distance and angle to reach
 * desired coordinate by taking in current position information and running set
 * calculations on the info to solve for needed variables.
 * @author Max Brodeur
 * @author Carl ElKhoury
 * @author Zakaria Essadaoui
 **/

public class Navigation {
	// Setting required fields
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 200;
	private static final double TILE_SIZE = 30.48;
	public OdometerData odometerData;
	private static final double radius = Lab5.getRadius();
	private static final double track = Lab5.getTrack();

	private static double deltaX;
	private static double deltaY;
	public static double current[];
	boolean stat = false;
	// boolean obstacle =false; //boolean that tells navigation there is an obstacle
	// ahead and pauses the travelTo() method.
	private static EV3LargeRegulatedMotor leftMotor = Lab5.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Lab5.getRightMotor();
	boolean finished360 = false;

	public double x;
	public double y;

	/**
	 * This method makes the robot travel to a (x,y) without checking for a ring
	 * @param x
	 * @param y
	 * @return void
	 */
	void travelTo(double x, double y) {
		travelTo(x, y, false);
	}
	/**This is the overloaded travel to method
	 * This method makes the robot travel to a (x,y) while checking for a ring
	 * @param x
	 * @param y
	 * @return void
	 */
	void travelTo(double x, double y, boolean obstacle) {

		this.x = x;
		this.y = y;
		// settign motor acceleration
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) { 																						// left motor.
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		current = Lab5.odometer.getXYT(); // gets current X Y and Theta values
		deltaX = x * TILE_SIZE - current[0]; // deltaX or deltaY is the difference between where you want to go and
												// where you are currently.
		deltaY = y * TILE_SIZE - current[1];
		// System.out.println(deltaX +" " +deltaY);
		double newTheta;
		if (deltaY == 0) { // series of checks to avoid division by 0 if destination coordinate is on same
							// axis
			if (deltaX < 0)
				newTheta = -Math.PI / 2;
			else
				newTheta = Math.PI / 2;
		} else {

			newTheta = Math.atan(deltaX / deltaY);

			if (deltaY < 0) {
				newTheta += Math.PI;
			}

		}
		turnTo(newTheta);

		stat = true; // boolean to say travelTo class is in action and moving forward. Very important
						// for the obstacleAvoid.
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		Sound.beep();
		leftMotor.rotate(convertDistance(radius, Math.sqrt(deltaX * deltaX + deltaY * deltaY)), true); // drives to
																										// coordinate
		rightMotor.rotate(convertDistance(radius, Math.sqrt(deltaX * deltaX + deltaY * deltaY)), obstacle);
		Sound.beep();
		// If we wan't to check for rings
		if (obstacle == true) { 
			int ring;
			// checks if there is a ring in front of the robot or in the search area
			while (leftMotor.isMoving() || rightMotor.isMoving()) {
				// System.out.println("Checking for the ring");
				try {
					Thread.sleep(40);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				// check for ring the search are
				Lab5.search.look();
				// check for rings in front of the robot
				ring = Lab5.search.detect.detect();
			    /*
			     * If the right ring is found
			     * Beep andGo to the upper right
			     * Otherwise, Beep twice, Avoid it and finished the course
			     */
				if (ring == (Lab5.TR - 1)) {
					Lab5.search.ringValue = ring;
					Sound.beep();
					// Avoid();
					travelTo(Lab5.UUX, Lab5.UUY);

				} else if (ring != 5) {
					Sound.beep();
					Sound.beep();
					Avoid();
					travelTo(x, y, true);
				}
			}

			// after the obstacle has been passed, restart the travelTo function recursively
			// to resume desired path.
		}
		stat = false; // set to false to indicate that we've reached the current coordinate and the
						// robot is stopped.
	}

	/**
	 * This method makes the robot avoid the ring in front of it
	 * @return void
	 */
	void Avoid() {
		move(-5);
		turnBy(-90);
		move(25);
		turnBy(90);
		move(45);
		turnBy(90);
		move(25);
		turnBy(-90);
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
	 * This method make the robot turn to a certain angle with a minimal angle
	 * @param theta
	 */
	void turnTo(double theta) {
		current = Lab5.odometer.getXYT();
		double deltaT = theta - Math.toRadians(current[2] % 360);
		deltaT %= 2 * Math.PI;
		if (deltaT > Math.PI)
			deltaT -= 2 * Math.PI;
		else if (deltaT < -Math.PI)
			deltaT += 2 * Math.PI;

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		deltaT %= Math.PI * 2; // ensures robot doesn't over turn.
		leftMotor.rotate(convertAngle(radius, track, deltaT * 180 / Math.PI), true);
		rightMotor.rotate(-convertAngle(radius, track, deltaT * 180 / Math.PI), false);

	}

	/**
	 * instructs the robot to sping 360 degrees in a direction specified by the
	 * boolean argument. A boolean true argument would result in clockwise 360
	 * degree turn.
	 **/

	void turn360(boolean clockWise) {
		double angle = 360;
		if (!clockWise)
			angle = -angle;
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(radius, track, angle), true);
		rightMotor.rotate(-convertAngle(radius, track, angle), true);
	}
	/**
	 * This method return weither the robot is navigating
	 * @return boolean
	 */
	boolean isNavigating() { // boolean created to indicate that travelTo() is currently running.
		return stat;
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
