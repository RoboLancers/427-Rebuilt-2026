// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

/** Add your docs here. */
public class IntakeShooterCommands {
  private Feeder m_feeder;
  private IntakeShooter m_IntakeShooter;
  
  public IntakeShooterCommands(Feeder m_feeder, IntakeShooter m_IntakeShooter) {
      this.m_feeder = m_feeder;
    this.m_IntakeShooter = m_IntakeShooter;
  }

  public Command Intake() {
    return m_IntakeShooter
        .set(FuelConstants.IntakingIntake)
        .alongWith(m_feeder.set(FuelConstants.IntakingFeeder));
  }

  public Command Eject() {
    return m_IntakeShooter
        .set(FuelConstants.EjectingIntake)
        .alongWith(m_feeder.set(FuelConstants.EjectingFeeder));
  }

  public Command Launch() {
    return m_IntakeShooter
        .set(FuelConstants.LaunchingIntake)
        .alongWith(m_feeder.set(FuelConstants.LaunchingFeeder));
  }

  public Command Stop() {
    return m_IntakeShooter
        .set(FuelConstants.StoppingIntake)
        .alongWith(m_feeder.set(FuelConstants.StoppingFeeder));
  }

  public Command SpinUp() {
    return m_IntakeShooter.set(FuelConstants.SpinupIntake);
  }
}
