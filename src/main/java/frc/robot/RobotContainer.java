package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  private final Feeder m_feeder = new Feeder();

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  boolean isCompetition = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...

  private final Field2d field = new Field2d();

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY() * DriveConstants.MAX_SPEED,
              () -> -m_driverController.getLeftX() * DriveConstants.MAX_SPEED)
          .withControllerRotationAxis(m_driverController::getRightX)
          .deadband(DriveConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY(),
              () -> -m_driverController.getLeftX())
          .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
          .deadband(DriveConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("SHOOT", timedCommand(Launch(), 1));
    NamedCommands.registerCommand("INTAKE", timedCommand(Intake(), 1));
    NamedCommands.registerCommand("OUTTAKE", timedCommand(Eject(), 1));
    NamedCommands.registerCommand("END_INTAKE", Stop());
    // NamedCommands.registerCommand("CLIMB", );

    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    SmartDashboard.putData("Field", field);

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          field.setRobotPose(pose);
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          field.getObject("target pose").setPose(pose);
        });

    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          field.getObject("path").setPoses(poses);
        });
  }

  // path.preventFlipping = true;
  public Command Intake() {
    return m_IntakeShooter
        .set(FuelConstants.IntakingIntake)
        .alongWith(m_feeder.set(FuelConstants.IntakingFeeder));
  }

  public Command Eject() {
    return m_IntakeShooter
        .set(FuelConstants.EjectingIntake)
        .alongWith(m_feeder.set(FuelConstants.EjectingFeeder));
  }

  public Command Launch() {
    return m_IntakeShooter
        .set(FuelConstants.LaunchingIntake)
        .alongWith(m_feeder.set(FuelConstants.LaunchingFeeder));
  }

  public Command Stop() {
    return m_IntakeShooter
        .set(FuelConstants.StoppingIntake)
        .alongWith(m_feeder.set(FuelConstants.StoppingFeeder));
  }

  public Command SpinUp() {
    return m_IntakeShooter.set(FuelConstants.SpinupIntake);
  }

  public Command timedCommand(Command command, double time) {
    return command.withTimeout(time);
  }

  private void configureBindings() {

    if (RobotBase.isSimulation()) {
      drivebase.resetPose(new Pose2d(2, 2, new Rotation2d()));
    }
    if (IntakeShooter.FuelCounter >= 10) {
      Stop();
    } else {
      m_driverController.leftBumper().whileTrue(Intake());
    }

    m_driverController
        .rightBumper()
        .whileTrue(
            SpinUp()
                .withTimeout(FuelConstants.SpinUpTime)
                .andThen(Launch())
                .finallyDo(() -> Stop()));
    m_driverController.a().whileTrue(Eject());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on
    // release

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Change this one
    } else {
      // sets default commands and other commands depending on mode
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

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

      if (DriverStation.isTest()) {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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
      autoChooser = AutoBuilder.buildAutoChooser();
      // AutoBuilder.buildAutoChooserWithOptionsModifier(
      //     (stream) ->
      //         isCompetition ? stream.filter(auto -> auto.getName().startsWith("comp")) :
      // stream);
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
