package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is implemented as a state machine thread and it provides an interface for driving the
 * robot around a certain set of points on a map.
 * 
 * @author angelortiz, paulattara
 *
 */
public class Navigation implements Runnable {

  // Class constants
  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 150;
  private static final double TILE_SIZE = 30.48;
  private static final int NAVIGATION_PERIOD = 50;
  private static final int BANDCENTER = 25; // Offset from the wall for BANG-BANG Controller (cm)
  private static final int BANDWIDTH = 5; // Width of dead band (cm)

  // Class attributes
  // Motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private EV3MediumRegulatedMotor sensorMotor;

  // Information about the robot and target
  private Odometer odometer;
  private UltrasonicPoller usPoller;
  private PController wallFollower;
  private int[] target;
  private double wheelRadius;
  private double track;

  // State machine flags
  private boolean directionChanged;
  private boolean isNavigating;
  private boolean obstacleMode;
  private boolean enableObstacleDetection;
  private double targetObstacleAngle;

  /**
   * Constructor for the Navigation class.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param sensorMotor
   * @param usPoller
   * @param wheelRadius
   * @param track
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      EV3MediumRegulatedMotor sensorMotor, UltrasonicPoller usPoller, double wheelRadius,
      double track, boolean enableObstacleDetection) throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.wallFollower = new PController(leftMotor, rightMotor, BANDCENTER, BANDWIDTH);
    this.usPoller = usPoller;

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.sensorMotor = sensorMotor;

    this.wheelRadius = wheelRadius;
    this.track = track;

    target = new int[2];
    target[0] = -1;
    target[1] = -1;
    isNavigating = false;
    directionChanged = false;
    obstacleMode = false;
    this.enableObstacleDetection = enableObstacleDetection;
  }

  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Main navigator state machine flow

      // In obstacle mode the motor controls are given to the wall follower P-type controller
      if (obstacleMode) {
        wallFollower.processUSData(usPoller.distance);
        double currAngle = odometer.getXYT()[2];
        // If the robot has reached the target angle (roughly turning 180 degrees) exit obstacle
        // mode
        if (currAngle < targetObstacleAngle + 10 && currAngle > targetObstacleAngle - 50) {
          obstacleMode = false;
          sensorMotor.rotate(45); // Reset sensor position
        }
      }

      // Enter obstacle mode is the distance reading from the odometer is less than 15cm
      if (enableObstacleDetection && !obstacleMode && usPoller.distance < 15
          && usPoller.distance > 0) {
        obstacleMode = true;
        directionChanged = true;
        turnToRelative(90); // Puts robot sideways with respect to the obstacle
        sensorMotor.rotate(-45); // Turns ultrasonic sensor 45 degrees to perform wall following
        targetObstacleAngle = (odometer.getXYT()[2] + 180) % 360; // sets target to exit obstacle
                                                                  // mode
      }

      // If the direction has changed recompute the trajectory of the robot
      if (!obstacleMode && directionChanged) {
        goToTarget();
        isNavigating = true;
        directionChanged = false;
      }

      // Set this flag to let other threads know that the robot is currently reaching a waypoint
      if (!obstacleMode && (!leftMotor.isMoving() && !rightMotor.isMoving()))
        isNavigating = false;

      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < NAVIGATION_PERIOD) {
        try {
          Thread.sleep(NAVIGATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Sets current target and indicates state machine to change direction.
   * 
   * @param x
   * @param y
   */
  public void travelTo(int x, int y) {
    this.target[0] = x;
    this.target[1] = y;

    directionChanged = true;
    isNavigating = true;
  }

  /**
   * Turns to an absolute angle ensuring minimal rotation.
   * 
   * @param theta
   */
  public void turnTo(double theta) {
    double currTheta = odometer.getXYT()[2];
    double targetRotation = 0;
    int directionAdjustment = 1; // 1 for right turn, -1 for left turn

    // Ensure that the minimal turn is taken
    if (theta < currTheta) {
      targetRotation = currTheta - theta;
      if (targetRotation < 180)
        directionAdjustment = -1;
      else
        targetRotation = 360 - targetRotation;
    } else {
      targetRotation = theta - currTheta;
      if (targetRotation > 180) {
        targetRotation = 360 - targetRotation;
        directionAdjustment = -1;
      }
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(wheelRadius, track, targetRotation) * directionAdjustment, true);
    rightMotor.rotate(-convertAngle(wheelRadius, track, targetRotation) * directionAdjustment,
        false);
  }

  /**
   * Turns to an relative angle ensuring minimal rotation.
   * 
   * @param theta
   */
  public void turnToRelative(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(wheelRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(wheelRadius, track, theta), false);
  }

  /**
   * Indicates whether the robot is still navigating.
   * 
   * @return
   */
  public boolean isNavigating() {
    return isNavigating;
  }

  /**
   * Moves the robot in the direction of the current target.
   */
  private void goToTarget() {
    // Compute the target's absolute angle and the distance required to reach it
    double[] position = odometer.getXYT();
    double[] realTarget =
        computeRealTarget(position[0], position[1], target[0] * TILE_SIZE, target[1] * TILE_SIZE);

    // Turn to target angle
    turnTo(realTarget[1]);

    // Move forward the required target distance
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(wheelRadius, realTarget[0]), true);
    rightMotor.rotate(convertDistance(wheelRadius, realTarget[0]), true);
  }

  /**
   * Computes the absolute angle and distance in cm required to reach the target with respect to the
   * current position.
   * 
   * @param currX
   * @param currY
   * @param targetX
   * @param targetY
   * @return
   */
  private double[] computeRealTarget(double currX, double currY, double targetX, double targetY) {
    double deltaX = targetX - currX;
    double deltaY = targetY - currY;
    int quadrant = 0;
    double[] computedTarget = new double[2];

    // Determine the quadrant of the target with respect to the current position
    if (deltaX >= 0 && deltaY >= 0)
      quadrant = 1;
    else if (deltaX >= 0 && deltaY < 0)
      quadrant = 2;
    else if (deltaX < 0 && deltaY < 0)
      quadrant = 3;
    else if (deltaX < 0 && deltaY >= 0)
      quadrant = 4;

    // Distance to the target
    double distance = Math.hypot(deltaX, deltaY);

    // Compute the absolute angle of direction to the target
    deltaX = Math.abs(deltaX);
    deltaY = Math.abs(deltaY);

    double targetTheta = 0;
    switch (quadrant) {
      case 1:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        break;
      case 2:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 90;
        break;
      case 3:
        targetTheta = Math.toDegrees(Math.atan(deltaX / deltaY));
        targetTheta += 180;
        break;
      case 4:
        targetTheta = Math.toDegrees(Math.atan(deltaY / deltaX));
        targetTheta += 270;
        break;
    }

    computedTarget[0] = distance;
    computedTarget[1] = targetTheta;

    return computedTarget;
  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
