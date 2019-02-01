package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {
  /**
   * The PController class provides a simple implementation of the Proportional Controller for the
   * Wall Following program. The main feature in the Proportional Controller is that is adjusts the
   * speeds of the wheels proportionally to the error in distance from the wall. The functionality
   * is provided through the UltrasonicController interface.
   * 
   * @author angelortiz paulattara
   */

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final int MAX_SPEED = 200;
  private static final int MIN_SPEED = 100;
  private static final double SPEED_OFFSET = 1.0;
  
  private final EV3LargeRegulatedMotor leftMotor;
  private final EV3LargeRegulatedMotor rightMotor;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;


  public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    //leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    //rightMotor.setSpeed(MOTOR_SPEED);
    //leftMotor.forward();
    //rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    /**
     * The processUSData method takes the distance measurement from the wall, and adjusts the
     * wheels' speeds and directions so that the cart follows the right direction.
     */

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

    // TODO: process a movement based on the us distance passed in (P style)

    int SCALE = 1; // P-Constant

    int delta = bandCenter - this.distance;
    int correction = Math.abs(delta) * SCALE;

    if (Math.abs(delta) < bandWidth) {
      // Below bandwidth, keep going straight
      leftMotor.setSpeed(MOTOR_SPEED);
      rightMotor.setSpeed(MOTOR_SPEED);
      leftMotor.forward();
      rightMotor.forward();

    } else if (delta < 0) {
      // Cart is too far
      // Decrease inside wheel rotation (left)
      if (leftMotor.getSpeed() > MIN_SPEED) {
        leftMotor.setSpeed(leftMotor.getSpeed() - correction);
      }
      if (leftMotor.getSpeed() > MIN_SPEED) {
        leftMotor.setSpeed(MIN_SPEED);
      }

      // Increase outside wheel rotation (right)
      if (rightMotor.getSpeed() < MAX_SPEED) {
        rightMotor.setSpeed(rightMotor.getSpeed() + correction);
      }
      if (rightMotor.getSpeed() > MAX_SPEED) {
        rightMotor.setSpeed(MAX_SPEED);
      }
      leftMotor.forward();
      rightMotor.forward();

    } else if (delta > 0) {
      // Cart is too close
      // Increase inside wheel rotation (left)
      if (leftMotor.getSpeed() < MAX_SPEED * SPEED_OFFSET) {
        leftMotor.setSpeed(leftMotor.getSpeed() + correction);
      }
      if (leftMotor.getSpeed() > MAX_SPEED * SPEED_OFFSET) {
        leftMotor.setSpeed((int) (MAX_SPEED * SPEED_OFFSET));
      }

      // Increase outside wheel rotation and reverse direction (right)
      if (rightMotor.getSpeed() < MIN_SPEED) {
        rightMotor.setSpeed(rightMotor.getSpeed() + correction);
      }
      if (rightMotor.getSpeed() > MIN_SPEED) {
        rightMotor.setSpeed(MIN_SPEED);
      }
      leftMotor.forward();
      rightMotor.backward();

    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
