package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


public class RobotContainer {

  public Command getAutonomousCommand() {
  try{
  } catch (Exception e) {
    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  }
        return Commands.none();
  }

  private final SendableChooser<Command> autoChooser;
  
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    private final Field2d field = new Field2d();
  
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

     SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                                m_driverController::getRightY)
                                                              .headingWhile(true);

  
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
 
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                           .withControllerHeadingAxis(() -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                                                      () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                                                           .headingWhile(true)
                                                                           .translationHeadingOffset(true)
                                                                           .translationHeadingOffset(Rotation2d.fromDegrees(0));
                                                            
  
  public RobotContainer() {

    boolean isCompetition = true;

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerve = new Swerve();
    exampleSubsystem = new ExampleSubsystem();

    NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
    new EventTrigger("shoot note").and(new Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot note");

    new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));

    autoCommand = new PathPlannerAuto("Example Auto");

    autoCommand.isRunning().onTrue(Commands.print("Example Auto started"));
    autoCommand.timeElapsed(5).onTrue(Commands.print("5 seconds passed"));
    autoCommand.timeRange(6, 8).whileTrue(Commands.print("between 6 and 8 seconds"));
    autoCommand.event("Example Event Marker").onTrue(Commands.print("passed example event marker"));
    autoCommand.pointTowardsZone("Speaker").onTrue(Commands.print("aiming at speaker"));
    autoCommand.activePath("Example Path").onTrue(Commands.print("started following Example Path"));
    autoCommand.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
    autoCommand.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4)).whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));

    configureButtonBindings();
    
    configureBindings();

     field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
      
List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
);

PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); 
PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, 
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
);

path.preventFlipping = true;
List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");
    return autoChooser.getSelected();

PPHolonomicDriveController.overrideXFeedback(() -> {
    return 0.0;
});
PPHolonomicDriveController.clearXFeedbackOverride();

PPHolonomicDriveController.overrideYFeedback(() -> {
    return 0.0;
});
PPHolonomicDriveController.clearYFeedbackOverride();

PPHolonomicDriveController.overrideRotationFeedback(() -> {
    return 0.0;
});
PPHolonomicDriveController.clearRotationFeedbackOverride();

PPHolonomicDriveController.clearFeedbackOverrides();
PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");
PathPlannerPath exampleChoreoTrajSplit = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj", 1);
  }


  private void configureBindings() {
  
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard); {


    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,0,0, new Constraints(5, 2)),
                                           new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360),
                                                                                            Units.degreesToRadians(180))
                                           ));
      m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.a().whileTrue(drivebase.sysIdDriveMotorCommand());
      m_driverController.b().whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); 

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }
    }
    }
  }