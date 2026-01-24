package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Robot Physical Constants & Speed
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(15); // this used to be like 14 or smth
  public static final double MAX_ANGULAR_SPEED = 15;

  public static class OperatorConstants {

    // Joystick Deadbband

    public static final int kDriverControllerPort = 0;
    public static final String drivebase = "drivebase";
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT =
        6; // this used to be 6 if turning speed is stupid make it six
  }

  public static class IntakeConstants {

    public static final int Intake_SparkMax_ID = 14;
    public static final int IntakenumMotors = 1;

    public static final int Intake_RPM = 60;
    public static final int FlyWheel_Diameter = 2;
    public static final double FlyWheel_Mass = 0.029;
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
    public static final double Intake_GearRatio = 12;
    public static final double x_DutyCycle = 0.3;
    public static final double y_DutyCycle = -0.3;

    public static final double DebounceTime = 0.1;
    public static final double DebounceMagnitude = 40;
    public static final double ClosedLoopRampRate = 0.25;
    public static final double OpenLoopRampRate = 0.25;
  }

  public static class FeederConstants {

    public static final int FeederdeviceId = 15;
    public static final int FeedernumMotors = 1;

    public static final int ClosedLoopControllerkP = 4;
    public static final int ClosedLoopControllerkI = 0;
    public static final int ClosedLoopControllerkD = 0;
    public static final int ClosedLoopControllerDegreesPerSec = 100;
    public static final int ClosedLoopControllerDegreesPerSecPerSec = 90;

    public static final int SoftLimitDegree = -30;
    public static final int SoftLimitDegreeMagnitude = 100;

    public static final int GearingreductionStages = 3;

    public static final int kP = 50;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int DegPerSecmagnitude = 90;
    public static final int DegPerSecPerSecmagnitude = 45;
    public static final int controllerAmagnitude = 60;
    public static final int controllerBmagnitude = 300;

    /** Need info for below */
    public static final int ks = 0;

    public static final int kv = 0;
    public static final int ka = 0;
    public static final int reductionStages = 12;
    public static final int StatorLimit = 40;

    public static final double ClosedLoopRampRate = 0.25;
    public static final double OpenLoopRampRate = 0.25;

    public static final double debouncerTime = 0.1;

    public static final int StatorAmps = 40;

    /** Need info for below */
    public static final int Diameter = 4;

    public static final double Mass = 0.3;
    public static final int UpperSoftLimit = 1000;

    public static final double controllerxdutyCycle = 0.3;
    public static final double controllerydutyCycle = -0.3;

    /** Need info for below */
  }

  public static class FuelConstants {

    public static final int SpinUpTime = 1;

    public static final int FuelLimit = 8;

    public static final double IntakingFeeder = 1.0;
    public static final double IntakingIntake = 1.0;

    public static final double EjectingFeeder = -0.9;
    public static final double EjectingIntake = -0.9;

    public static final double LaunchingFeeder = -0.9;
    public static final double LaunchingIntake = 0.8;

    public static final int StoppingFeeder = 0;
    public static final int StoppingIntake = 0;

    public static final double SpinupIntake = 0.4;
  }
}
