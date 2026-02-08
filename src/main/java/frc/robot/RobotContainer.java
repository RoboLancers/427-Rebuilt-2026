package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Led.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Field2d field = new Field2d();

  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY() * Constants.DriveConstants.MAX_SPEED,
              () -> -m_driverController.getLeftX() * Constants.DriveConstants.MAX_SPEED)
          .withControllerRotationAxis(
              () ->
                  m_driverController.getRightX()
                      * Constants.DriveConstants.MAX_ANGULAR_SPEED) // ASDFGHJKL
          .deadband(Constants.DriveConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(
              () -> -m_driverController.getRightY() * Constants.DriveConstants.MAX_ANGULAR_SPEED,
              () ->
                  -m_driverController.getRightX()
                      * Constants.DriveConstants.MAX_ANGULAR_SPEED) // ASDFGHJKL
          .headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a robotRelative input stream.

  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY() * Constants.DriveConstants.MAX_SPEED,
              () -> -m_driverController.getLeftX() * Constants.DriveConstants.MAX_SPEED)
          .withControllerRotationAxis(
              () ->
                  m_driverController.getRawAxis(2)
                      * Constants.DriveConstants.MAX_ANGULAR_SPEED) // ASDFGHJKL
          .deadband(Constants.DriveConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2), // X axis
              () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2)) // Y axis
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on
    // release

  }

  // sets default commands and other commands depending on mode
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
  Command driveFieldOrientedDirectAngleKeyboard =
      drivebase.driveFieldOriented(driveDirectAngleKeyboard);
  Command driveFieldOrientedAnglularVelocityKeyboard =
      drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

  {
    if (RobotBase.isSimulation()) {
      drivebase.resetPose(new Pose2d(2, 2, new Rotation2d()));
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Change this one
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngle.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      m_driverController
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.a().whileTrue(drivebase.sysIdDriveMotorCommand());
      m_driverController
          .b()
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngle.driveToPoseEnabled(true), // And this one
                  () -> driveDirectAngle.driveToPoseEnabled(false))); // And this one
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController
          .leftBumper()
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // Configure to run auto

  }
}
