package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.Lab5.Lab5;
import ca.mcgill.ecse211.odometer.OdometerData;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ObstacleDetect implements UltrasonicController{

	private static final int FILTER_OUT = 20;
	private int filterControl;
	public int distance;
	private static final double Radius = Lab5.getRadius();
	private static final double track = Lab5.getTrack();
	public int lastDistance;

	/** continuously polls ultrasonic Sensor for distance, if distance threshold is met, avoid the obstacle through the hardcoded
	 * square turn around the block. Also decides whether to perform the turn on the left or right depending on readings from the 
	 * Ultrasonic sensor after the first 90 degree turn. **/
	@Override
	public void processUSData(int distance) {
		lastDistance = this.distance;
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
	}


	public int readUSDistance() {
		return this.distance;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}


}
