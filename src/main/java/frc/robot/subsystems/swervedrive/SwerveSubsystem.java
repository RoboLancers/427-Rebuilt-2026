// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.





package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;




public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
  private final SwerveDrive  swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(File directory) {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
    swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
       
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
 
        //Make the robot move
        swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                            headingX.getAsDouble(),
                            headingY.getAsDouble(),
                            swerveDrive.getOdometryHeading().getRadians(),
                            swerveDrive.getMaximumChassisVelocity()));
  });
}
  /**
   * Command to drive the robot using translative values and heading as angular velocity
   * 
   * @param translationX       Translation in the X direction
   * @param translationY       Translation in the Y direction
   * @param angularRotationX   Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
  return run(() -> {
    //Make the robot move
    swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                                        true,
                                        false);
  });
  }
}
