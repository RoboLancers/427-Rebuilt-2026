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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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


//This is the main class for the swerve drive subsystem 
public class SwerveSubsystem extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    public SwerveSubsystem() {

      RobotConfig config;
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }

      setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getCurrentSpeeds(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getCurrentModuleStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
    }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
    }
  }
 

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
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                Constants.robotConfig, // The robot configuration
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
   * 
   * IT DOES SOMETHING
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
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
}

public Command centerModulesCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'centerModulesCommand'");
}
public ChassisSpeeds getTargetSpeeds(double asDouble, double asDouble2, Rotation2d rotation2d) {
  // TODO Auto-generated method stub
  throw new UnsupportedOperationException("Unimplemented method 'getTargetSpeeds'");
}



  

