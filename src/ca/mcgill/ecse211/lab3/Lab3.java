// Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Class containing the main method, uses all the class interfaces to make the cart drive around a
 * set of waypoints.
 * 
 * @author angelortiz
 *
 */
public class Lab3 {
  
  private static boolean ENABLE_OBSTACLE_DETECTION = true;

  // Test cases
  public static final int[][] MAP1 = {{0, 2}, {1, 1}, {2, 2}, {2, 1}, {1, 0}};
  public static final int[][] MAP2 = {{1, 1}, {0, 2}, {2, 2}, {2, 1}, {1, 0}};
  public static final int[][] MAP3 = {{1, 0}, {2, 1}, {2, 2}, {0, 2}, {1, 1}};
  public static final int[][] MAP4 = {{0, 1}, {1, 2}, {1, 0}, {2, 1}, {2, 2}};

  // Motor objects
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();

  // Sensor objects
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the
                                                                               // instance
  private static final SampleProvider usDistance = usSensor.getMode("Distance");

  // Parameters
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 15.2;
  public static final int[][] WAYPOINTS = MAP3;

  public static void main(String[] args) throws OdometerExceptions {

    lcd.clear();
    lcd.drawString(" Press any button ", 0, 2);
    lcd.drawString("     to start.    ", 0, 3);

    Button.waitForAnyPress();

    // Odometer and navigation related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Display odometryDisplay = new Display(lcd);
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance);
    Navigation navigation =
        new Navigation(leftMotor, rightMotor, sensorMotor, usPoller, WHEEL_RAD, TRACK, ENABLE_OBSTACLE_DETECTION);

    // Start odometer and display threads
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
    Thread usPollerThread = new Thread(usPoller);
    usPollerThread.start();
    Thread navigationThread = new Thread(navigation);
    navigationThread.start();

    for (int[] waypoint : WAYPOINTS) {
      navigation.travelTo(waypoint[0], waypoint[1]);
      while (true) {
        try {
          Thread.sleep(70);
        } catch (InterruptedException e) {
        }
        if (!navigation.isNavigating())
          break;
      }
    }

    System.exit(0);
  }
}
