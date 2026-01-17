// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// four score and seven years ago, our fathers brought forth unto this continent, a new nation, conceived in Liberty, and dedicated to the proposition that all men are created equal. 
// Now we are engaged in a great civil war, testing whether that nation, or any nation so conceived and so dedicated, can long endure. We are met on a great battlefield of that war. We have come to dedicate a portion of that field, as a final resting place for those who here gave their lives that that nation might live. It is altogether fitting and proper that we should do this.
// But, in a larger sense, we cannot dedicate -- we cannot consecrate -- we cannot hallow -- this ground. The brave
// men, living and dead, who struggled here, have consecrated it, far above our poor power to add or detract. The world will little note, nor long remember what we say here, but it can never forget what they did here. It is for us the living, rather, to be dedicated here to the unfinished work which they who fought here have thus far so nobly advanced. It is rather for us to be here dedicated to the great task remaining before us -- that from these honored dead we take increased devotion to that cause for which they gave the last full measure of devotion -- that we here highly resolve that these dead shall not have died in vain -- that this nation, under God, shall have a new birth of freedom -- and that government of the people, by the people, for the people, shall not perish from the earth.
// -- Abraham Lincoln, Gettysburg Address
// -- November 19, 1863
// I think I made a good choice including this speech here. - Copilot
// Also, if you read this far, you're awesome. - Copilot
// I love you. - Copilot
// Thank you for using our services. - Copilot
// Have a great day! - Copilot
// Yesterday, december 2nd, 44 B.C.,Gaius Iulius Caesar,dictator for life,was assassinated by a group of senators including Marcus Iunius Brutus and Gaius Cassius Longinus. 
// So let it be written, so let it be done. - Julius Caesar
// Veni, vidi, vici. - Julius Caesar
// I came, I saw, I conquered. - Julius Caesar
// Et tu, Brute? - Julius Caesar
// You too, Brutus? - Julius Caesar
// Now go forth, and build something amazing. - Copiolot
// Seriously, you're awesome. - Copiolot
// Thank you for reading this far. - Copilot
// This is the main class for the swerve drive subsystem 

//Everyone is a genius. But if you judge a fish by its ability to climb a tree, it will live its whole life believing that it is stupid. - Albert Einstein
//Imagination is more important than knowledge. For knowledge is limited, whereas imagination embraces the entire world, stimulating progress, giving birth to evolution. - Albert Einstein
//Life is like riding a bicycle. To keep your balance, you must keep moving. - Albert Einstein
//Try not to become a man of success, but rather try to become a man of value. - Albert Einstein
//The important thing is not to stop questioning. Curiosity has its own reason for existing. - Albert Einstein
//Science without religion is lame, religion without science is blind. - Albert Einstein
//If you can't explain it simply, you don't understand it well enough. - Albert Einstein
//Logic will get you from A to B. Imagination will take you everywhere. - Albert Einstein
//Anyone who has never made a mistake has never tried anything new. - Albert Einstein
//The true sign of intelligence is not knowledge but imagination. - Albert Einstein
//The only source of knowledge is experience. - Albert Einstein
//Education is what remains after one has forgotten what one has learned in school. - Albert Einstein
//I have no special talent. I am only passionately curious. - Albert Einstein
//Great spirits have always encountered violent opposition from mediocre minds. - Albert Einstein
//The difference between stupidity and genius is that genius has its limits. - Albert Einstein
//Peace cannot be kept by force; it can only be achieved by understanding. - Albert Einstein
// Imagination is everything. It is the preview of life's coming attractions. - Albert Einstein
// Everything should be made as simple as possible, but not simpler. - Albert Einstein
// Two things are infinite: the universe and human stupidity; and I'm not sure about the universe. - Albert Einstein
package frc.robot.subsystems.swervedrive;

//Imports! D:
import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
  private final SwerveDrive  swerveDrive;


  /* Creates a new SwerveSubsystem. */
  public SwerveSubsystem(File directory) {
    //File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    //Catches any errors within the code and crashes the program if there are any
       /* DO NOT TOUCH
       Yknow what, dont touch in generalI dont know
              what this part does and none of us should
              |
              V   PUT A AN END HER */
    try {
    swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);   
    }
      // This method will be called once per scheduler run during simulation

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
   public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Command centerModulesCommand() {
      return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                      scaledInputs.getY(),
                                                      angle.getRadians(),
                                                      getHeading().getRadians(),
                                                      Constants.MAX_SPEED);
  }



}