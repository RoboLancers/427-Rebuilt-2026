// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

//Imports! D:
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


//This is the main class for the swerve drive subsystem 
public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
  private final SwerveDrive  swerveDrive;

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;


  /* Creates a new SwerveSubsystem. */
  public SwerveSubsystem(File directory) {
    //File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    //Catches any errors within the code and crashes the program if there are any
       /* DO NOT TOUCH
       Yknow what, dont touch in generalI dont know
              what this part does and none of us should
              |
              V    */ 
    try {
    swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);   
    }
      // This method will be called once per scheduler run during simulation

     RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

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
    // This method will be called once per scheduler run
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
 
        //Make the robot move
        //Mat
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
    //More Mat
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
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
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

  public Command driveFieldOriented(SwerveInputStream driveDirectAngle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveFieldOriented'");
  }

public Object resetOdometry(Pose2d pose2d) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
}

public Command centerModulesCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'centerModulesCommand'");
}



  }

