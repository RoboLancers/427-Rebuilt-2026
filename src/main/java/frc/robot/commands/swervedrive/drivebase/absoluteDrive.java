// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class absoluteDrive extends Command {
  /** Creates a new absoluteDrive. */
  private final SwerveSubsystem swerve;

  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private boolean initRotation = false;

  public absoluteDrive(
      SwerveSubsystem swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingHorizontal,
      DoubleSupplier headingVertical) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds desiredSpeeds =
        swerve.getTargetSpeeds(
            vX.getAsDouble(),
            vY.getAsDouble(),
            headingHorizontal.getAsDouble(),
            headingVertical.getAsDouble());

    // Prevent Movement After Auto
    if (initRotation) {
      if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
        // Get current heading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set current heading to desired heading
        desiredSpeeds =
            swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      // Dont init rotation agin
      initRotation = false;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation =
        SwerveMath.limitVelocity(
            translation,
            swerve.getFieldVelocity(),
            swerve.getPose(),
            Constants.LOOP_TIME,
            Constants.ROBOT_MASS,
            List.of(Constants.CHASSIS),
            swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot Move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
