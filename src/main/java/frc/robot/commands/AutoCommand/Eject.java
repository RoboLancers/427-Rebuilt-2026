// Copyright (c) FIRST and other WPILIB contributors.
// Open Source Softwarel youc an modify and/or share it under the terms of
// the WPILIB BSD liscense file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

public class Eject extends Command {
  IntakeShooter intakeShooter;
  Feeder feeder;

  public Eject(IntakeShooter intakeShooter, Feeder feeder) {
    this.feeder = feeder;
    addRequirements(intakeShooter);
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
