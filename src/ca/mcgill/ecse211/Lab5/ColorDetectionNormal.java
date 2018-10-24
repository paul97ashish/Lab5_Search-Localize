package ca.mcgill.ecse211.Lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to detect the color of the ring that is in front of the
 * sensor after each button press
 * 
 * @author Team 21
 *
 */
public class ColorDetectionNormal {
	// Defining our sensor and lcd Display
	static Port portColor = LocalEV3.get().getPort("S1");
	static EV3ColorSensor colorSensor = new EV3ColorSensor(portColor);
	static float[] sampleColor;
	static SampleProvider colorValue;
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	// Defining the normalized mean for RGB values for each ring int the following order
	// Blue, Green, Yellow, Orange
	private static final float[][] Meann = { { 0.170390332f, 0.767597595f, 0.617868163f },
											 { 0.402231815f, 0.906190081f, 0.13049561f },
											 { 0.832694447f, 0.538629888f, 0.128443766f },
											 { 0.953786617f, 0.290982684f, 0.074967764f } };
	// this was our normalized standard deviation from our data collection
	// Since the values we got were too big 
	// we decide to use a small value for the standard deviation 
	// to have no range overlapping
	
	// private static final float[] [] StdDevn= {{0.573214593f,0.606117356f, 0.551404371f},
	// {0.584991691f,0.583430374f,0.563377067f},
	// {0.584095753f,0.585842632f,0.561801176f},
	// { 0.58910641f,0.579596568f,0.563046585f}};

	public static void main(String[] args) {
		// setting the right mode for the sensor
	    colorSensor.setFloodlight(lejos.robotics.Color.WHITE);
		colorValue = colorSensor.getMode("RGB");
		sampleColor = new float[colorValue.sampleSize()];
		// waiting for a press from the user and display the ring color
		while (true) {
			Button.waitForAnyPress();
			lcd.clear();
			int status;
			// keep checking until a known color is detected
			do {
				status = findMatch(fetch());
			} while (status == 5);
			// display a message with the color detected
			String[] str = { "Blue", "Green", "Yellow", "Orange" };
			lcd.drawString("Object Detected ", 0, 0);
			lcd.drawString(str[status], 0, 1);

		}
	}

	/**
	 * This method gets the data from the sensor
	 * @return sample color: array with the RGB values
	 */
	public static float[] fetch() {
		colorValue.fetchSample(sampleColor, 0);
		return sampleColor;
	}
	/**
	 * This method return the corresponding color for the RGB values that were passed to it
	 * If the RGB doesn't correspond to any known colors, it return 5
	 * @param array: RGB values from the sensor
	 * @return index of the color deteted
	 */
	public static int findMatch(float array[]) {
		// System.out.println(array[0]+ " "+array[1]+ " "+array[2]+ " ");
		// Standardizing our RGB values
		float euc = (float) Math.sqrt((Math.pow(array[0], 2) + Math.pow(array[1], 2) + Math.pow(array[2], 2)));
		float R = array[0] / euc;
		float G = array[1] / euc;
		float B = array[2] / euc;
		// checking if the RGB value correspond to any color
		
		for (int i = 0; i < 4; i++) {
			// we use 0.05 as a standard deviation
			float differenceR = Math.abs(R - Meann[i][0]) / 0.05f;
			float differenceG = Math.abs(G - Meann[i][1]) / 0.05f;
			float differenceB = Math.abs(B - Meann[i][2]) / 0.05f;
			// if a match is found, return the index of the color
			if (differenceR < 1.0 && differenceG < 1.0 && differenceB < 1.0) {
				return i;
			}
		}
		return 5;
	}
}
