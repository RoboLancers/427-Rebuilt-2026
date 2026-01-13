package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants {
    public static final int kP = 50;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int DegPerSecmagnitude = 90;
    public static final int DegPerSecPerSecmagnitude = 45;
    public static final int controllerAmagnitude = 60;
    public static final int controllerBmagnitude = 300;

    /**
     * Need info for below
     */

    public static final int ks = 0;
    public static final int kv = 0;
    public static final int ka = 0;
    public static final int reductionStages = 0;
    public static final int StatorLimit = 40;
    
    /**
     * Need info for below
     */

    public static final int Diameter = 4;
    public static final int Mass = 1;
    public static final int UpperSoftLimit = 1000;

    public static final double controllerxdutyCycle = 0.3;
    public static final double controllerydutyCycle = -0.3;

    /**
     * Need info for below
     */
    
    public static final int shooterdeviceId = 4;

    public static final int numMotors = 1;

  }
}
