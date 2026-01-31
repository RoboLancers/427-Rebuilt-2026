package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import choreo.auto.AutoChooser;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
@Logged
public class RobotContainer {

  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
  }
  boolean isCompetition = true;

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
  
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
    private final Field2d field = new Field2d();
  
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> -m_driverController.getLeftY() * Constants.MAX_SPEED,
                                                                  () -> -m_driverController.getLeftX() * Constants.MAX_SPEED)
                                                              .withControllerRotationAxis(m_driverController::getRightX)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

     SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                                m_driverController::getRightY)
                                                              .headingWhile(true);
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
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
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //private Swerve swerve = new Swerve();
    final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    //NamedCommands.registerCommand("autoBalance", drivebase.autoBalanceCommand());
    //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    //new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
    //new EventTrigger("shoot note").and(new Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot note"));

    //new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));
    
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

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
try {
  List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");
} catch (IOException e) {
  e.printStackTrace();
} catch (ParseException e) {
  e.printStackTrace();
}

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
try {
  PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");
} catch (FileVersionException e) {
  e.printStackTrace();
} catch (IOException e) {
  e.printStackTrace();
} catch (ParseException e) {
  e.printStackTrace();
}
try {
  PathPlannerPath exampleChoreoTrajSplit = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj", 1);
} catch (FileVersionException e) {
  e.printStackTrace();
} catch (IOException e) {
  e.printStackTrace();
} catch (ParseException e) {
  e.printStackTrace();
}
  }

  private void configureBindings() {

    if(RobotBase.isSimulation()){
      drivebase.resetPose(new Pose2d(2,2,new Rotation2d()));
    }
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   // new Trigger(m_exampleSubsystem::exampleCondition)
     //   .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard); {

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); //Change this one
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngle.driveToPose(() -> target,
                                           new ProfiledPIDController(5,0,0, new Constraints(5, 2)),
                                           new ProfiledPIDController(5,0,0, new Constraints(Units.degreesToRadians(360),
                                                                                            Units.degreesToRadians(180))
                                           ));
      m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.a().whileTrue(drivebase.sysIdDriveMotorCommand());
      m_driverController.b().whileTrue(Commands.runEnd(() -> driveDirectAngle.driveToPoseEnabled(true), //And this one
                                                     () -> driveDirectAngle.driveToPoseEnabled(false))); //And this one

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
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */