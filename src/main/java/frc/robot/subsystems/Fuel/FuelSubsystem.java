package frc.robot.subsystems.Fuel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

public class FuelSubsystem extends SubsystemBase {
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  private final Feeder m_feeder = new Feeder();

  public FuelSubsystem() {}

  public Command intake() {
    return m_feeder
        .set(FuelConstants.IntakingFeeder)
        .alongWith(m_IntakeShooter.set(FuelConstants.IntakingIntake));
    // wheels spinning inward
  }

  public void eject() {
    m_feeder.set(FuelConstants.EjectingFeeder * -1);
    m_IntakeShooter.set(FuelConstants.EjectingIntake * -1);
    // wheels spinning outward, to feed human player or ferry
  }

  public void launch() {
    m_feeder.set(FuelConstants.LaunchingFeeder);
    m_IntakeShooter.set(FuelConstants.LaunchingIntake);
    // wheels spinning inwards to score
  }

  public void stop() {
    m_feeder.set(FuelConstants.StoppingFeeder);
    m_IntakeShooter.set(FuelConstants.StoppingIntake);
    // stop
  }

  public void spinUp() {
    m_IntakeShooter.set(FuelConstants.SpinupIntake);
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_feeder.updateTelemetry();
    // m_IntakeShooter.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // m_feeder.simIterate();
  }
}
