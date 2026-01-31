// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

// Imports! D:
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
// import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

// This is the main class for the swerve drive subsystem
public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private SwerveDrive swerveDrive;

  /* Creates a new SwerveSubsystem. */
  public SwerveSubsystem(File directory) {
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    // Catches any errors within the code and crashes the program if there are any
    /* DO NOT TOUCH
    Yknow what, dont touch in general I dont know
           what this part does and none of us should
           |
           V    */

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Configure AutoBuilder last
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      setpointGenerator = new SwerveSetpointGenerator(config, Units.rotationsToRadians(10.0));

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(0.0, 0.0, 1.2), new PIDConstants(0.01, 0.0, 0.1)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);

    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  @Override
  public void simulationPeriodic() {}

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          // Mat
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity
   *
   * @param translationX Translation in the X direction
   * @param translationY Translation in the Y direction
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   *     <p>IT DOES SOMETHING
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          // More Mat
          swerveDrive.drive(
              new Translation2d(
                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public void drive(Translation2d translation, double Rotation, boolean fieldReletive) {
    swerveDrive.drive(translation, Rotation, fieldReletive, false);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.MAX_SPEED);
  }
}
