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
  public static class IntakeConstants {
    public static final int Intake_RPM = 1;
    public static final int FlyWheel_Diameter = 4;   //get info
    public static final int FlyWheel_Mass = 1;      //get info
    public static final int SoftLimit = 1000;

    public static final int KP = 50;    
    public static final int KI = 0;
    public static final int KD = 0;
    public static final int ks = 0;
    public static final int kv = 0;
    public static final int ka = 0;

    public static final int CurrentLimit = 40;
    public static final int MaxVelocity = 90;
    public static final int MaxAcceleration = 45;
    public static final double Intake_GearRatio = 12;  //get info
    public static final double x_DutyCycle = 0.3;
    public static final double y_DutyCycle = -0.3;

    public static final double DebounceTime = 0.1;
    public static final double DebounceMagnitude = 40;
  }

  public static class MotorConstants {
    public static final int Intake_SparkMax_ID = 4;
    public static final int numMotors = 1; 
  }
}
