package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class RingSearch  {
	private static final EV3MediumRegulatedMotor SensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor leftMotor = Lab5.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Lab5.getRightMotor();
	private static ColorDetection detect;
	private static int distance;
	private static int Last;
	private static UltrasonicPoller usPoller;
	private static double LastX, LastY;
	private static Navigation navigation;
	private static final int SPEED = 50;
	int ringValue = 5;
	private static double radius = Lab5.WHEEL_RAD;
	boolean notFound= true;

	public RingSearch(UltrasonicPoller poller, Navigation navig) {
		//System.out.println("RingSerch created");
		SensorMotor.setSpeed(100);
		SensorMotor.rotate(-110);
		detect = new ColorDetection();
		usPoller=poller;
		navigation = navig;
	}

	public void look() {
		/*Last = usPoller.getDistance();
		System.out.println(Last);*/
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		distance = usPoller.getDistance();
		System.out.println(distance);
		if (distance<50) {
			leftMotor.stop(true);
			rightMotor.stop();
			LastX = Lab5.odometer.getXYT()[0];
			LastY = Lab5.odometer.getXYT()[1];
			double TargX = navigation.x;
			double TargY = navigation.y;
		//	if (checkRing()) {
			
				int distance = usPoller.getDistance();
				System.out.println("rechecked distance :" + distance);
				
				double theta=Lab5.odometer.getXYT()[2];
				if(theta<45 && theta> 315) {
					navigation.travelTo((LastX + distance - 15)/Lab5.TILE_SIZE, LastY/Lab5.TILE_SIZE);
				}else if(theta>45 && theta<135) {
					navigation.travelTo((LastX )/Lab5.TILE_SIZE, (LastY + distance - 15)/Lab5.TILE_SIZE);
				}else if(theta<225 && theta> 135) {
					navigation.travelTo((LastX - distance + 15)/Lab5.TILE_SIZE, LastY/Lab5.TILE_SIZE);
				}else {
					navigation.travelTo((LastX )/Lab5.TILE_SIZE, (LastY - distance  +15)/Lab5.TILE_SIZE);
				}
				
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
					notFound=false;
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
				navigation.travelTo(LastX/Lab5.TILE_SIZE, LastY/Lab5.TILE_SIZE);
				leftMotor.rotate(convertAngle(radius, Lab5.TRACK, 90),true);
				rightMotor.rotate(-convertAngle(radius, Lab5.TRACK, 90),false);
				leftMotor.rotate(convertDistance(radius,13), true);
				rightMotor.rotate(convertDistance(radius,13),false);
				navigation.travelTo(TargX, TargY, notFound);

		}

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

	private static int convertDistance(double radius, double distance) {	//converts distance to wheel rotations
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {	//converts angle to radians for degree rotation
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
