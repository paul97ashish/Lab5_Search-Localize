package ca.mcgill.ecse211.Lab5;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * We used this class to test our ultrasonic sensor and see the noise that it outputs
 * We also used to tune our track value in order to get accurate rotation of the robot
 * @author Group 21
 *
 */
public class Test {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	// this instance
	private static float[] usData = new float[usDistance.sampleSize()];
	private static final int  Rotate_Speed=100;

	private static final double TRACK=15.6;
	private static final double WHEEL_RAD=2.2;
	private static final int FILTER_OUT = 20;
	public static void main (String[] args) throws InterruptedException  {
		leftMotor.setSpeed(Rotate_Speed);
		rightMotor.setSpeed(Rotate_Speed);
		while (true) {
			System.out.println(fetch());
			Thread.sleep(2000);
		}
	}
	public static int fetch() {
		int sensdistance=0;
		while (sensdistance==0) {
			usDistance.fetchSample(usData, 0); 
			
		int distance=(int) (usData[0] * 100.0);
				
		int filterControl=0;
		 if (distance >= 255 && filterControl < FILTER_OUT) {
	          // bad value, do not set the distance var, however do increment the
	          // filter value
	    	  
	          filterControl++;
	        } else if (distance >= 255) {
	          // We have repeated large values, so there must actually be nothing
	          // there: leave the distance alone
	          distance=255;
	          sensdistance=distance;
	         
	        } else {
	          // distance went below 255: reset filter and leave
	          // distance alone.
	          filterControl = 0;
	          sensdistance=distance;
	         
	        }
		}
		return sensdistance;
	}
	/**
	 * This method calculate the angle that must be passed to the motor 
	 * using the radius of the wheel and the distance we want the robot to cross
	 * @param radius
	 * @param distance
	 * @return	corresponding distance
	 */
	private static int convertDistance(double radius, double distance) {
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
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
