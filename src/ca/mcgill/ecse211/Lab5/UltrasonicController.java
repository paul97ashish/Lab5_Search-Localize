package ca.mcgill.ecse211.Lab5;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
