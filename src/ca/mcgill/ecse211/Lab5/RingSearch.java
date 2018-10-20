package ca.mcgill.ecse211.Lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class RingSearch extends Thread{
	private static final EV3MediumRegulatedMotor SensorMotor=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
	private static EV3LargeRegulatedMotor leftMotor = Lab5.getLeftMotor();
	private static EV3LargeRegulatedMotor rightMotor = Lab5.getRightMotor();
	private static ColorDetection detect;
	private static int distance;
	private static int Last;
	private static ObstacleDetect sensor;
	private static double LastX, LastY;
	private static Navigation navigation;
	private static final int SPEED=50;
	int ringValue=5;
	public RingSearch (ObstacleDetect sens, Navigation navig) {
		SensorMotor.rotate(90);
		detect=new ColorDetection();
		sensor=sens;
		navigation=navig;
	}
	
	public void run() {
		Last=sensor.readUSDistance();
		while (Lab5.navigation.isNavigating()) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			distance=sensor.readUSDistance();
			
			if ((Last-distance)>50) {
				leftMotor.stop(true);
				rightMotor.stop();
				LastX=Lab5.odometryDisplay.getXYT()[0];
				LastY=Lab5.odometryDisplay.getXYT()[1];
				if(checkRing()) {
					int distance=sensor.readUSDistance();
					navigation.travelTo(LastX+distance-10, LastY);
					leftMotor.setSpeed(SPEED);
					rightMotor.setSpeed(SPEED);
					ringValue=detect.detect();
					while (ringValue==5) {
						leftMotor.forward();
						rightMotor.forward();
					}
					
					
				}
				
			}
			
			
		}
		
	}
	public boolean checkRing() {
		int distance1=sensor.readUSDistance();
		int distance2=sensor.readUSDistance();
		int distance3=sensor.readUSDistance();
		return (distance1<100)&&(distance2<100)&&(distance3<100);
	}
	
	public int ringValue() {
		return ringValue+1;
	}
	
}
