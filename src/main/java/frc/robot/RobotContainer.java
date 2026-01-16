package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }

  private final SendableChooser<Command> autoChooser;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    boolean isCompetition = true;

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerve = new Swerve();
    exampleSubsystem = new ExampleSubsystem();

    NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
    new EventTrigger("shoot note").and(new Trigger(exampleSubsystem::someCondition)).onTrue(Commands.print("shoot note");

    new PointTowardsZoneTrigger("Speaker").whileTrue(Commands.print("aiming at speaker"));

    autoCommand = new PathPlannerAuto("Example Auto");

    autoCommand.isRunning().onTrue(Commands.print("Example Auto started"));
    autoCommand.timeElapsed(5).onTrue(Commands.print("5 seconds passed"));
    autoCommand.timeRange(6, 8).whileTrue(Commands.print("between 6 and 8 seconds"));
    autoCommand.event("Example Event Marker").onTrue(Commands.print("passed example event marker"));
    autoCommand.pointTowardsZone("Speaker").onTrue(Commands.print("aiming at speaker"));
    autoCommand.activePath("Example Path").onTrue(Commands.print("started following Example Path"));
    autoCommand.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
    autoCommand.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4)).whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));

    configureButtonBindings();
    
    // Configure the trigger bindings
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
