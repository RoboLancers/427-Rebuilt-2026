package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ClimbConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.AutoCommand.Eject;
import frc.robot.commands.AutoCommand.Intake;
import frc.robot.commands.AutoCommand.Shoot;
import frc.robot.commands.Drive;
import frc.robot.subsystems.CANDriveSubsystem;
// import frc.robot.subsystems.Climb.ClimbSubsystem;
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
  private CANDriveSubsystem driveSubsystem; // = new CANDriveSubsystem();
  // private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  boolean isCompetition = true;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);

  private SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...

  private final Field2d field = new Field2d();

  SwerveSubsystem drivebase;

  SwerveInputStream driveAngularVelocity;

  SwerveInputStream driveAngularVelocityKeyboard;
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard;

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented;

  SwerveInputStream driveDirectAngle;

  // Clone's the angular velocity input stream and converts it to a robotRelative input stream.

  // Derive the heading axis with math!

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putNumber("Far Shooter Speed", FuelConstants.SpinUpIntakeFar);
    SmartDashboard.putNumber("Close Shooter Speed", FuelConstants.SpinUpIntakeClose);

    if (IsSwerve) {
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

      SwerveInputStream aimWhileDriving =
          SwerveInputStream.of(
                  drivebase.getSwerveDrive(),
                  () -> -m_driverController.getLeftY() * Constants.DriveConstants.MAX_SPEED,
                  () -> -m_driverController.getLeftX() * Constants.DriveConstants.MAX_SPEED)
              .withControllerRotationAxis(
                  () -> m_driverController.getRightX() * Constants.DriveConstants.MAX_ANGULAR_SPEED)
              .deadband(DEADBAND)
              .scaleTranslation(0.8)
              .allianceRelativeControl(true)
              .aim(FieldConstants.BLUE_HUB)
              .aimWhile(m_driverController.y());

      driveAngularVelocity =
          SwerveInputStream.of(
                  drivebase.getSwerveDrive(),
                  () -> m_driverController.getLeftY(),
                  () -> m_driverController.getLeftX())
              .withControllerRotationAxis(() -> m_driverController.getRightX())
              .deadband(DEADBAND)
              .scaleTranslation(0.8)
              .allianceRelativeControl(true);

      aimWhileDriving =
          driveAngularVelocity.copy().aim(FieldConstants.BLUE_HUB).aimWhile(m_driverController.y());

      driveAngularVelocityKeyboard =
          SwerveInputStream.of(
                  drivebase.getSwerveDrive(),
                  () -> -m_driverController.getLeftY(),
                  () -> -m_driverController.getLeftX())
              .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
              .deadband(DEADBAND)
              .scaleTranslation(0.8)
              .allianceRelativeControl(true);
      // Derive the heading axis with math!
      driveDirectAngleKeyboard =
          driveAngularVelocityKeyboard
              .copy()
              .withControllerHeadingAxis(
                  () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                  () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
              .headingWhile(true)
              .translationHeadingOffset(true)
              .translationHeadingOffset(Rotation2d.fromDegrees(0));

      /**
       * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
       */
      SwerveInputStream driveRobotOriented =
          driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

      driveDirectAngle =
          driveAngularVelocity
              .copy()
              .withControllerHeadingAxis(
                  () ->
                      -m_driverController.getRightY() * Constants.DriveConstants.MAX_ANGULAR_SPEED,
                  () ->
                      -m_driverController.getRightX()
                          * Constants.DriveConstants.MAX_ANGULAR_SPEED) // ASDFGHJKL
              .headingWhile(true);
    } else {
      driveSubsystem = new CANDriveSubsystem();
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand(
        "SHOOT", SpinUpClose().withTimeout(1).andThen(Shoot()).withTimeout(3));
    NamedCommands.registerCommand("SHOOT_FAR", new Shoot(m_IntakeShooter, m_feeder).withTimeout(3));
    NamedCommands.registerCommand("INTAKE", new Intake(m_IntakeShooter, m_feeder));
    NamedCommands.registerCommand("OUTTAKE", new Eject(m_IntakeShooter, m_feeder));
    NamedCommands.registerCommand("OUTTAKE_2", new Eject(m_IntakeShooter, m_feeder).withTimeout(3));
    NamedCommands.registerCommand("END_INTAKE", Stop());
    NamedCommands.registerCommand("WAIT", new WaitCommand(2.5));
    NamedCommands.registerCommand(
        "DEPLOY", Commands.none()); // timedCommand(m_ClimbSubsystem.setDeployAngle(), 1));
    // NamedCommands.registerCommand("CLIMB", );
    NamedCommands.registerCommand("CLOSE_SHOOT", Commands.none());
    new EventTrigger("INTAKE_EVENT")
        .whileTrue(
            m_IntakeShooter
                .setVelocity(RPM.of(FuelConstants.IntakingIntake))
                .alongWith(m_feeder.setVelocity(RPM.of(FuelConstants.IntakingFeeder))));

    configureBindings();

    m_IntakeShooter.setDefaultCommand(m_IntakeShooter.set(0));
    m_feeder.setDefaultCommand(m_feeder.set(0));

    // m_IntakeShooter.setDefaultCommand(m_IntakeShooter.ManualSpeedControl());

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

    // Set the default command to force the arm to go to 0.
    //   m_ClimbSubsystem.setDefaultCommand(
    //       m_ClimbSubsystem.setAngle(Degrees.of(ClimbConstants.DefaultAngle)));
  }

  public void updateVisionSim() {}

  // path.preventFlipping = true;
  public Command Intake() {
    return m_IntakeShooter
        .setVelocity(RPM.of(FuelConstants.IntakingIntake))
        .alongWith(m_feeder.setVelocity(RPM.of(FuelConstants.IntakingFeeder)));
  }

  public Command Eject() {
    return m_IntakeShooter
        .setVelocity(RPM.of(FuelConstants.EjectingIntake))
        .alongWith(m_feeder.setVelocity(RPM.of(FuelConstants.EjectingFeeder)));
  }

  public Command Shoot() {
    return m_feeder
        .setVelocity(RPM.of(FuelConstants.IntakingFeeder))
        .withTimeout(0.5)
        .andThen(m_feeder.setVelocity(RPM.of(FuelConstants.LaunchingFeeder)));
  }

  public Command Stop() {
    return m_IntakeShooter
        .setVelocity(RPM.of(FuelConstants.StoppingIntake))
        .alongWith(m_feeder.setVelocity(RPM.of(FuelConstants.StoppingFeeder)));
  }

  public Command SpinUpClose() {
    // return m_IntakeShooter.setVelocity(RPM.of(FuelConstants.SpinUpIntakeClose));
    return m_IntakeShooter.ManualSpeedControl();
  }

  public Command SpinUpFar() {
    return m_IntakeShooter.setVelocity(RPM.of(FuelConstants.SpinUpIntakeFar));
  }

  public Command timedCommand(Command command, double time) {
    return command.withTimeout(time);
  }

  public Command shootAuto() {
    return SpinUpFar().alongWith(Commands.waitSeconds(1).andThen((Shoot()))).withTimeout(4);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via then
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.leftBumper().whileTrue(Intake());
    m_driverController.rightBumper().whileTrue(SpinUpClose());
    m_driverController.rightTrigger().whileTrue(SpinUpFar());
    m_driverController.leftTrigger().whileTrue(Eject());
    m_driverController.y().whileTrue(Shoot());
    m_driverController.x().whileTrue(Stop());
    if (!Constants.OperatorConstants.IsSwerve) {
      driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, m_driverController));
    }

    if (RobotBase.isSimulation()) {
      drivebase.resetPose(new Pose2d(2, 2, new Rotation2d()));
    }

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on
    // release
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    if (IsSwerve) {
      Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveAngularVelocity);
      Command driveFieldOrientedAnglularVelocity =
          drivebase.driveFieldOriented(driveAngularVelocity);
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

      if (Robot.isSimulation()) {
        Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
        // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
        driveDirectAngle.driveToPose(
            () -> target,
            new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
            new ProfiledPIDController(
                5,
                0,
                0,
                new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
        m_driverController
            .start()
            .onTrue(
                Commands.runOnce(
                    () -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        // m_driverController.a().whileTrue(drivebase.sysIdDriveMotorCommand());
        // m_driverController
        //     .b()
        //     .whileTrue(
        //         Commands.runEnd(
        //             () -> driveDirectAngle.driveToPoseEnabled(true), // And this one
        //             () -> driveDirectAngle.driveToPoseEnabled(false))); // And this one
      }
      if (DriverStation.isTest()) {
        drivebase.setDefaultCommand(
            driveFieldOrientedAnglularVelocity); // Overrides drive command above!

        m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      } else {
        m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        m_driverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        // m_driverController
        //     .leftTrigger()
        //     .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        // m_driverController.rightBumper().onTrue(Commands.none());
        // m_driverController.b().whileTrue(m_ClimbSubsystem.set(0.8));
        // m_driverController.y().whileTrue(m_ClimbSubsystem.set(-0.8));
      }
    }
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("do Nothing", null);
    // AutoBuilder.buildAutoChooserWithOptionsModifier(
    //     (stream) ->
    //         isCompetition ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
