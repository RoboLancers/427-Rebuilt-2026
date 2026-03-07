package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public static class DriveConstants { // Robot Physical Constants & Speed
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag}

    public static final double MAX_SPEED =
        Units.feetToMeters(15); // this used to be like 14 or smth
    public static final double MAX_ANGULAR_SPEED = Units.feetToMeters(15);

    public static final double TANK_SPEED_MULTIPLYER = 0.1;
    public static final double TANK__TURNING_SPEED_MULTIPLYER = 0.1;
    // Joystick deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 30;
    public static final int LEFT_FOLLOWER_ID = 40;
    public static final int RIGHT_LEADER_ID = 20;
    public static final int RIGHT_FOLLOWER_ID = 62;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static class ClimbConstants {
    // ClimbSubsystem
    public static final double kP = 0.4;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int MaxVelocity = 90;
    public static final int MaxAcceleration = 45;

    public static final double ks = 0;
    public static final double kg = 0.015;
    public static final double kv = 12.19;
    public static final double ka = 0.23;

    public static final int GearRatio = 100;

    public static final boolean MotorInverted = false;

    public static final int StatorCurrentLimit = 40;
    public static final double LoopRampRate = 0.25;

    public static final int SparkMaxDeviceID = 17;

    public static final int NumMotors = 1;
    // NumMotors and SoftLowerLimit will have to be adjusted based on the actual climb arm's freedom
    // of movement.
    public static final int SoftLowerLimit = -50;
    public static final int SoftUpperLimit = 110;
    public static final int HardMin = -50;
    public static final int HardMax = 110;
    public static final int StartingPosition = 90;

    public static final int Length = 3;
    public static final int Mass = 3;

    public static final int ToleranceAngle = 5;
    public static final int DeployAngle = 0;
    public static final int ClimbAngle = 110;

    public static final int MaximumVoltage = 12;
    public static final int StepVoltage = 2;
    public static final int Duration = 4;

    // RobotContainer
    public static final int DefaultAngle = 90;
    public static final int A_Angle = 20;
    public static final int B_Angle = -20;
    public static final double X_DutyCycle = 0.3;
    public static final double Y_DutyCycle = -0.3;
  }

  public static class OperatorConstants {

    // Joystick Deadbband

    // Joystick Deadbband

    public static final int kDriverControllerPort = 0;
    public static final String drivebase = "drivebase";
    public static final double DEADBAND = 0.3;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    // this used to be 6 if turning speed is stupid make it six

    public static final int DRIVE_SCALING = 7;
    public static final int ROTATION_SCALING = 6;

    public static final boolean IsSwerve = true;
  }

  public static class VisionConstants {
    public static final boolean isVision = false;

    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from cen

    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    ;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class IntakeConstants {

    public static final int Intake_SparkMax_ID = 15;
    public static final int IntakeFollower_SparkMax_ID = 16;
    public static final int IntakenumMotors = 1;

    public static final int Intake_RPM = 60;
    public static final int FlyWheel_Diameter = 4;
    public static final double FlyWheel_Mass = 4;
    public static final int SoftLimit = 5000;

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double ks = 1;
    public static final double kv = 0.187;
    public static final double ka = 0;

    public static final int CurrentLimit = 40;
    public static final int MaxVelocity = 90;
    public static final int MaxAcceleration = 45;
    public static final double Intake_GearRatio = 1.5; // /on swerve bot 2/3
    public static final double x_DutyCycle = 0.3;
    public static final double y_DutyCycle = -0.3;

    public static final double DebounceTime = 0.1;
    public static final double DebounceMagnitude = 40;
    public static final double ClosedLoopRampRate = 0.05;
    public static final double OpenLoopRampRate = 0.05;
  }

  public static class FeederConstants {

    public static final int FeederdeviceId = 14;
    public static final int FeedernumMotors = 1;

    public static final int ClosedLoopControllerkP = 4;
    public static final int ClosedLoopControllerkI = 0;
    public static final int ClosedLoopControllerkD = 0;
    public static final int ClosedLoopControllerDegreesPerSec = 100;
    public static final int ClosedLoopControllerDegreesPerSecPerSec = 90;

    public static final int SoftLimitDegree = -30;
    public static final int SoftLimitDegreeMagnitude = 100;

    public static final double GearingreductionStages = 1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int DegPerSecmagnitude = 90;
    public static final int DegPerSecPerSecmagnitude = 45;
    public static final int controllerAmagnitude = 60;
    public static final int controllerBmagnitude = 300;

    public static final double ks = 1.5;
    public static final double kv = 0.113;
    public static final double ka = 0;
    public static final int reductionStages = 1; // on swerve bot 1/3
    public static final int StatorLimit = 40;

    public static final double ClosedLoopRampRate = 0.05;
    public static final double OpenLoopRampRate = 0.05;

    public static final double debouncerTime = 0.1;

    public static final int StatorAmps = 40;

    public static final int Diameter = 4;

    public static final double Mass = 0.3;
    public static final int UpperSoftLimit = 5000;

    public static final double controllerxdutyCycle = 0.3;
    public static final double controllerydutyCycle = -0.3;
  }

  public static class FuelConstants {

    public static final int SpinUpTime = 1;

    public static final int FuelLimit = 8;

    public static final double IntakingFeeder = -4000;
    public static final double IntakingIntake = -3000;

    public static final double EjectingFeeder = 4000;
    public static final double EjectingIntake = 3000;

    public static final double LaunchingFeeder = 4000;
    public static final double LaunchingIntake = -3000;

    public static final int StoppingFeeder = 0;
    public static final int Stoppi
    public static final double SpinUpIntakeClose = -2000;
    public static final double SpinUpIntakeFar = -3000;
  }

  public static class CameraConstants {
    public static final Rotation3d FRONT_LEFT_ROTATION =
        new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30));
    public static final Translation3d FRONT_LEFT_TRANSLATION =
        new Translation3d(
            Units.inchesToMeters(11), Units.inchesToMeters(7.25), Units.inchesToMeters(9));

    public static final Rotation3d BACK_LEFT_ROTATION =
        new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(150));
    public static final Translation3d BACK_LEFT_TRANSLATION =
        new Translation3d(
            Units.inchesToMeters(-11), Units.inchesToMeters(7.25), Units.inchesToMeters(9));

    public static final Rotation3d FRONT_RIGHT_ROTATION =
        new Rotation3d(0, Units.degreesToRadians(-24.094), Math.toRadians(-30));
    public static final Translation3d FRONT_RIGHT_TRANSLATION =
        new Translation3d(
            Units.inchesToMeters(11), Units.inchesToMeters(-7.25), Units.inchesToMeters(9));

    public static final Rotation3d BACK_RIGHT_ROTATION =
        new Rotation3d(0, Units.degreesToRadians(-24.094), Math.toRadians(-150));
    public static final Translation3d BACK_RIGHT_TRANSLATION =
        new Translation3d(
            Units.inchesToMeters(-11), Units.inchesToMeters(-7.25), Units.inchesToMeters(9));
  }

  public static class FieldConstants {

    public static final Pose2d BLUE_HUB =
        new Pose2d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84),
            Rotation2d.fromDegrees(180));
    public static final double BLUE_HUB_X = BLUE_HUB.getX();
    public static final double BLUE_HUB_Y = BLUE_HUB.getY();

    public static final Pose2d BLUE_TOWER =
        new Pose2d(
            Units.inchesToMeters(27.00), Units.inchesToMeters(147.47), Rotation2d.fromDegrees(180));
    public static final double BLUE_TOWER_X = BLUE_TOWER.getX();
    public static final double BLUE_TOWER_Y = BLUE_TOWER.getY();

    public static final Pose2d BLUE_DEPOT =
        new Pose2d(
            Units.inchesToMeters(13.5), Units.inchesToMeters(234.78), Rotation2d.fromDegrees(180));
    public static final double BLUE_DEPOT_X = BLUE_DEPOT.getX();
    public static final double BLUE_DEPOT_Y = BLUE_DEPOT.getY();

    public static final Pose2d BLUE_CHUTE =
        new Pose2d(0, Units.inchesToMeters(26.22), Rotation2d.fromDegrees(180));
    public static final double BLUE_CHUTE_X = BLUE_CHUTE.getX();
    public static final double BLUE_CHUTE_Y = BLUE_CHUTE.getY();

    public static final Pose2d RED_HUB =
        new Pose2d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.84),
            Rotation2d.fromDegrees(180));
    public static final double RED_HUB_X = RED_HUB.getX();
    public static final double RED_HUB_Y = RED_HUB.getY();

    public static final Pose2d RED_TOWER =
        new Pose2d(
            Units.inchesToMeters(635.72),
            Units.inchesToMeters(170.22),
            Rotation2d.fromDegrees(180));
    public static final double RED_TOWER_X = RED_TOWER.getX();
    public static final double RED_TOWER_Y = RED_TOWER.getY();

    public static final Pose2d RED_DEPOT =
        new Pose2d(
            Units.inchesToMeters(637.72), Units.inchesToMeters(82.84), Rotation2d.fromDegrees(180));
    public static final double RED_DEPOT_X = RED_DEPOT.getX();
    public static final double RED_DEPOT_Y = RED_DEPOT.getY();

    public static final Pose2d RED_CHUTE =
        new Pose2d(
            Units.inchesToMeters(651.22),
            Units.inchesToMeters(291.47),
            Rotation2d.fromDegrees(180));
    public static final double RED_CHUTE_X = RED_CHUTE.getX();
    public static final double RED_CHUTE_Y = RED_CHUTE.getY();
  }
}
