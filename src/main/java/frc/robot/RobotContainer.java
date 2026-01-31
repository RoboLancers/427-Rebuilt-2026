package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  private final Feeder m_feeder = new Feeder();
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  // private final FuelSubsystem m_fuel = new FuelSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings

    configureBindings();

    autoChooser.setDefaultOption("Do Nothing", null);

    // m_IntakeShooter.setDefaultCommand(m_IntakeShooter.set(0));

    m_feeder.setDefaultCommand(m_feeder.set(0));
    m_IntakeShooter.setDefaultCommand(m_IntakeShooter.set(0));
    // m_fuel.setDefaultCommand(m_fuel.stopCommand());

    DriverStation.silenceJoystickConnectionWarning(true);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
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

  private void configureBindings() {
    if (IntakeShooter.FuelCounter >= 10) {
      Stop();
    } else {
      m_driverController.leftBumper().whileTrue(Intake());
    }

    m_driverController
        .rightBumper()
        .whileTrue(
            SpinUp()
                .withTimeout(FuelConstants.SpinUpTime)
                .andThen(Launch())
                .finallyDo(() -> Stop()));
    m_driverController.a().whileTrue(Eject());
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, m_driverController));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on
    // release

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // Configure to run auto
  }
}
