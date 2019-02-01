/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import java.util.Set;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.hardware.Sound;

/**
 * This class counts lines on the floor using a color sensor, and uses this count and the direction
 * of the cart as computed by the odometer to correct the X and Y values. The corrected value is
 * obtained by multiplying the amount of lines by the stored length of the tiles being traversed.
 * 
 * @author angelortiz, paulattara
 *
 */
public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_SIZE = 30.48;
  private static final double SENSOR_OFFSET = 3; // Distance between sensor and cart center
  private static final double LINE_COLOR_VALUE = 0.2; // Minimum value required to treat a sensor
                                                      // reading as a line
  private static final int SQUARE_SIZE = 4; // Amount of tiles per side on the square to drive
  private Odometer odometer;
  // Discrete locations are used to approximate the position the cart by counting lines in different
  // directions
  private int XE = 0; // X position when going East
  private int XW = SQUARE_SIZE - 1; // X position when going West
  private int YN = 0; // Y position when going North
  private int YS = SQUARE_SIZE - 1; // Y position when going South
  private int lineCount = 0; // Overall line count

  private static final Port usPort = LocalEV3.get().getPort("S1");

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }


  /**
   * Constantly checks color sensor readings to count lines. When a line is detected the values from
   * the odometer are overwritten with more accurate ones.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {

    EV3ColorSensor sensor = new EV3ColorSensor(usPort); // usSensor is the instance
    SampleProvider sensorProvider = sensor.getMode("Red");
    MeanFilter filter = new MeanFilter(sensorProvider, 4);
    float[] sample = new float[sensor.sampleSize()];
    boolean inLine = false; // Flags whether the sensor is currently seeing a line
    boolean makeCorrection = false; // Tells the class to correct the odometer values
    double[] position = odometer.getXYT(); // Current odometer values
    Direction direction = Direction.INIT; // Direction of the robot's movement

    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // Trigger correction (When do I have information to correct?)
      filter.fetchSample(sample, 0);
      boolean lineDetected = sample[0] < LINE_COLOR_VALUE;

      if (lineDetected) {
        if (!inLine) {
          inLine = true;
          makeCorrection = true;
          lineCount++;
          Sound.beep();
        }
      } else
        inLine = false;

      // Calculate new (accurate) robot position
      if (makeCorrection && lineCount < ((SQUARE_SIZE - 1) * 4)) {
        makeCorrection = false;
        position = odometer.getXYT();

        // Get the direction of movement according to the angle
        double theta = position[2];
        if (theta > 45 && theta < 135) {
          direction = Direction.EAST;
        } else if (theta > 135 && theta < 225) {
          direction = Direction.SOUTH;
        } else if (theta > 225 && theta < 315) {
          direction = Direction.WEST;
        } else {
          direction = Direction.NORTH;
        }

        // Update odometer with new calculated (and more accurate) vales
        double correctedX, correctedY;
        switch (direction) {
          case NORTH:
            correctedY = (YN * TILE_SIZE) - SENSOR_OFFSET;
            YN++;
            odometer.setXYT(position[0], correctedY, position[2]);
            // System.out.println("Correcting Y to: " + correctedY);
            break;
          case EAST:
            correctedX = (XE * TILE_SIZE) - SENSOR_OFFSET;
            XE++;
            odometer.setXYT(correctedX, position[1], position[2]);
            // System.out.println("Correcting X to: " + correctedX);
            break;
          case SOUTH:
            YS--;
            correctedY = (YS * TILE_SIZE) + SENSOR_OFFSET;
            odometer.setXYT(position[0], correctedY, position[2]);
            // System.out.println("Correcting Y to: " + correctedY);
            break;
          case WEST:
            XW--;
            correctedX = (XW * TILE_SIZE) + SENSOR_OFFSET;
            odometer.setXYT(correctedX, position[1], position[2]);
            // System.out.println("Correcting X to: " + correctedX);
            break;
        }
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }

  public enum Direction {
    NORTH, EAST, SOUTH, WEST, INIT
  }

}
