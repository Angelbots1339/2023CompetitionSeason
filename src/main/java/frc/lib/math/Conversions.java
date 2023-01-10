package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class Conversions {

  /**
   * @param counts    CANCoder Counts
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double CANcoderToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 4096.0));
  }

  /**
   * @param degrees   Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between CANCoder and Mechanism
   * @return CANCoder Counts
   */
  public static double degreesToCANcoder(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * 4096.0));
    return ticks;
  }

  /**
   * @param counts    Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees   Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param velocityCounts Falcon Position Counts
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return Total Rotations of Mechanism
   */
  public static double falconToRotaiton(double positionCounts, double gearRatio) {
    double motorR = positionCounts / 2048.0;
    double mechR = motorR / gearRatio;
    return mechR;
  }

  /**
   * @param RPM       RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
   *                  RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param positioncounts Falcon position Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
   *                       Falcon RPM)
   * @return Meters
   */
  public static double falconToMeters(double positioncounts, double circumference, double gearRatio) {
    double wheelR = falconToRotaiton(positioncounts, gearRatio);
    double wheelM = (wheelR * circumference);
    return wheelM;
  }

  /**
   * @param velocity      Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
   *                      Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param x position from joystick
   * @param y position from joystick
   * @return Rotation2d
   * 
   *         <pre>
      Joystick input:
               (y:-)
                 |
                 |
       (x:-)-----|------(x:+)
                 |
                 |
               (y:+)
  
   intial angle (-180 -> 180):
             -180/180°
               (x:-)
                 |
                 |
   -90°(y:-)-----|------(y:+) 90°
                 | ノ
                 |   θ
               (x:+)
                0°
  
  transformed(+180) desired angle (0 -> 360): same angle as gyro
                 0°
               (x:+)
             θ   |
               / |
    90°(y:+)-----|------(y:-)270°
                 | 
                 |  
               (x:-)
                180°
   *         </pre>
   */
  public static Rotation2d ConvertJoystickToAngle(double x, double y) {
    return new Rotation2d(Math.atan2(x, y) + Math.PI);
  }

  /**
   * 
   * Maps the joystick values from a circle to a square.
   * Otherwise the joystick hardware reads values in a square from (-1, -1) to (1,
   * 1), but because of the circular joystick shape you can never reach the
   * corners of this square.
   * 
   * @param returnAxis The axis desired to be converted
   * @param otherAxis  the other axsis (need for computation)
   * @return
   */
  public static Double mapJoystick(Double returnAxis, double otherAxis) {
    return returnAxis * Math.sqrt(1 - ((otherAxis * otherAxis) / 2));
  }
}