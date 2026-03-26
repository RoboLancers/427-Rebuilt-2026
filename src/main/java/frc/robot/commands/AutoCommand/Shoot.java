// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

/* You shouder using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  IntakeShooter intakeShooter;
  Feeder feeder;

  /** Creates a new Shoot. */
  public Shoot(IntakeShooter intakeShooter) {
    this.intakeShooter = intakeShooter;
    this.feeder = feeder;
    addRequirements(intakeShooter);
    addRequirements(feeder);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

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
    feeder.setVelocity(RPM.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
