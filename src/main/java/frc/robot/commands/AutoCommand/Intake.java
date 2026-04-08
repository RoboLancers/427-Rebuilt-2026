// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

public class Intake extends Command {
  IntakeShooter intakeShooter;
  Feeder feeder;

  // creates a new shoot
  public Intake(IntakeShooter intakeShooter, Feeder feeder) {
    this.intakeShooter = intakeShooter;
    this.feeder = feeder;
    addRequirements(intakeShooter);
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    intakeShooter.setVelocity(RPM.of(IntakeShooter.ShootSpeed));
    feeder.setVelocity(RPM.of(FuelConstants.IntakingFeeder));
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    feeder.setVelocity(RPM.of(FuelConstants.LaunchingFeeder));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.setVelocity(RPM.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
