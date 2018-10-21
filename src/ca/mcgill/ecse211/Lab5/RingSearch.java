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
	private static ObstacleDetect sensor;
	private static double LastX, LastY;
	private static Navigation navigation;
	private static final int SPEED = 50;
	int ringValue = 5;

	public RingSearch(ObstacleDetect sens, Navigation navig) {
		System.out.println("RingSerch created");
		SensorMotor.rotate(-90);
		detect = new ColorDetection();
		sensor = sens;
		navigation = navig;
	}

	public void look() {
		Last = sensor.readUSDistance()*100;
		System.out.println(Last);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		distance = sensor.readUSDistance()*100;

		if ((Last - distance) > 20) {
			leftMotor.stop(true);
			rightMotor.stop();
			LastX = Lab5.odometryDisplay.getXYT()[0];
			LastY = Lab5.odometryDisplay.getXYT()[1];
			double TargX = navigation.x;
			double TargY = navigation.y;
			if (checkRing()) {
				int distance = sensor.readUSDistance()*100;
				navigation.travelTo(LastX + distance - 10, LastY);
				leftMotor.setSpeed(SPEED);
				rightMotor.setSpeed(SPEED);
				ringValue = detect.detect();
				while (ringValue == 5) {
					leftMotor.forward();
					rightMotor.forward();
					ringValue = detect.detect();
				}
				if (ringValue == (Lab5.TR - 1)) {
					Sound.beep();
				} else {
					Sound.beep();
					Sound.beep();
				}
				leftMotor.setSpeed(200);
				rightMotor.setSpeed(200);
				leftMotor.backward();
				rightMotor.backward();
				navigation.travelTo(LastX, LastY);
				navigation.travelTo(TargX, TargY);
			}

		}

	}

	public boolean checkRing() {
		int distance1 = sensor.readUSDistance()*100;
		int distance2 = sensor.readUSDistance()*100;
		int distance3 = sensor.readUSDistance()*100;
		return (distance1 < 100) && (distance2 < 100) && (distance3 < 100);
	}

	public int ringValue() {
		return ringValue + 1;
	}

}
