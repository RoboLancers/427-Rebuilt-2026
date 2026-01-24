package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  //private final ExampleSubsystem m_subsystem;
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  private final Feeder m_feeder = new Feeder();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeShooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.exampleMethodCommand().schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.set(FuelConstants.LaunchingFeeder);
    m_IntakeShooter.set(FuelConstants.LaunchingIntake);
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
