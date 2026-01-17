// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

//Imports! D:
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    private SwerveDrive swerveDrive;
    public SwerveSubsystem() {

      RobotConfig config;
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }

      setpointGenerator = new SwerveSetpointGenerator(
            config,
            Units.rotationsToRadians(10.0)
        );

        ChassisSpeeds currentSpeeds = getCurrentSpeeds();
        SwerveModuleState[] currentStates = getCurrentModuleStates();
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
    }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */

    public void driveRobotRelative(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint,
            speeds,
            0.02
        );
        setModuleStates(previousSetpoint.moduleStates());
    }
  
 
  public SwerveSubsystem(File directory) {
    RobotConfig config; 
    try {
      config = RobotConfig.fromGUISettings();
    swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);

    } catch (Exception e) {
      throw new RuntimeException(e);   
    } 
    setpointGenerator = new SwerveSetpointGenerator(
      config,
      Units.rotationsToRadians(10.0)
    );
  }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */
    public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathCommand(
                path,
                this::getPose,
                this::getRobotRelativeSpeeds,
                this::drive,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                ),
                Constants.robotConfig,
                () -> {
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }   

  @Override
  public void simulationPeriodic() {
  }
  public void periodic() {
  }

  public Command sysIdDriveMotorCommand() {
      return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(),
           this, swerveDrive, 12, true),
           3.0, 5.0, 3.0);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
       
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
 
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
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
   
    swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                                        true,
                                        false);
  });
  }

    public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
  @param velocity Velocity according to the field.
   */

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX, headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }


  public void drive(Translation2d translation, double Rotation, boolean fieldReletive) {
    swerveDrive.drive(translation,
                      Rotation,
                      fieldReletive,
                      false);
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }
  public SwerveDriveConfiguration getSwerveDriveConfiguration(){ 

    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }
 
  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

public Object resetOdometry(Pose2d pose2d) {
    throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
}

public Command centerModulesCommand() {
    throw new UnsupportedOperationException("Unimplemented method 'centerModulesCommand'");
}

public ChassisSpeeds getTargetSpeeds(double asDouble, double asDouble2, Rotation2d rotation2d) {
  throw new UnsupportedOperationException("Unimplemented method 'getTargetSpeeds'");
}
}