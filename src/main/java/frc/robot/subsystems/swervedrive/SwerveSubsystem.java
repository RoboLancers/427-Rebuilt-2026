// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

// Imports! D:
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
<<<<<<< HEAD
import edu.wpi.first.math.util.Units;
=======
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

// This is the main class for the swerve drive subsystem
public class SwerveSubsystem extends SubsystemBase {
<<<<<<< HEAD
  double maximumSpeed = Units.feetToMeters(4.5);
=======
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
  private final SwerveDrive swerveDrive;

  /* Creates a new SwerveSubsystem. */
  public SwerveSubsystem(File directory) {
    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    // Catches any errors within the code and crashes the program if there are any
<<<<<<< HEAD
    /* DO NOT TOUCH
    Yknow what, dont touch in generalI dont know
           what this part does and none of us should
           |
           V    */
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
=======

    /* DO NOT TOUCH or everything breaks
    |
    V    */
    try {
      swerveDrive =
          new SwerveParser(directory).createSwerveDrive(Constants.DriveConstants.MAX_SPEED);
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    // This method will be called once per scheduler run during simulation

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

<<<<<<< HEAD
          // Make the robot move
          // Mat
=======
          // Make the robot move with math
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
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
<<<<<<< HEAD
   *     <p>IT DOES SOMETHING
=======
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
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
<<<<<<< HEAD
        Constants.MAX_SPEED);
=======
        Constants.DriveConstants.MAX_SPEED);
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
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
<<<<<<< HEAD
        Constants.MAX_SPEED);
=======
        Constants.DriveConstants.MAX_SPEED);
>>>>>>> f3b41229e09984beda9d92704500552def42a4cb
  }
}
