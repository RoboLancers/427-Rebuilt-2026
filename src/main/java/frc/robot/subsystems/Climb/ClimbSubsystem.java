package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              ClimbConstants.kP,
              ClimbConstants.kI,
              ClimbConstants.kD,
              DegreesPerSecond.of(ClimbConstants.MaxVelocity),
              DegreesPerSecondPerSecond.of(ClimbConstants.MaxAcceleration))
          .withSimClosedLoopController(
              ClimbConstants.kP,
              ClimbConstants.kI,
              ClimbConstants.kD,
              DegreesPerSecond.of(ClimbConstants.MaxVelocity),
              DegreesPerSecondPerSecond.of(ClimbConstants.MaxAcceleration))
          .withFeedforward(
              new ArmFeedforward(ClimbConstants.ks, ClimbConstants.kg, ClimbConstants.kv))
          .withSimFeedforward(
              new ArmFeedforward(ClimbConstants.ks, ClimbConstants.kg, ClimbConstants.kv))
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          .withGearing(ClimbConstants.GearRatio)
          .withMotorInverted(ClimbConstants.MotorInverted)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(ClimbConstants.StatorCurrentLimit))
          .withClosedLoopRampRate(Seconds.of(ClimbConstants.LoopRampRate))
          .withOpenLoopRampRate(Seconds.of(ClimbConstants.LoopRampRate));

  private SparkMax spark = new SparkMax(ClimbConstants.SparkMaxDeviceID, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(ClimbConstants.NumMotors), smcConfig);

  private Debouncer statorDebounce =
      new Debouncer(0.1); // Debouncer to prevent rapid changes in 0.1s

  // Game piece is detected if you're using over 40A current for more than 0.1s
  public boolean isGamePieceIn() {
    return statorDebounce.calculate(sparkSmartMotorController.getStatorCurrent().gte(Amps.of(40)));
  }

  private ArmConfig armCfg =
      new ArmConfig(sparkSmartMotorController)
          .withSoftLimits(
              Degrees.of(ClimbConstants.SoftLowerLimit), Degrees.of(ClimbConstants.SoftUpperLimit))
          .withHardLimit(Degrees.of(ClimbConstants.HardMin), Degrees.of(ClimbConstants.HardMax))
          .withStartingPosition(Degrees.of(ClimbConstants.StartingPosition))
          .withLength(Feet.of(ClimbConstants.Length))
          .withMass(Pounds.of(ClimbConstants.Mass))
          .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  private Arm arm = new Arm(armCfg);

  public ClimbSubsystem() {}

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   *
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return arm.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
   * setpoint.
   *
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle) {
    return arm.runTo(angle, Degrees.of(ClimbConstants.ToleranceAngle));
  }

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   *
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) {
    arm.setMechanismPositionSetpoint(angle);
  }

  /**
   * Move the arm up and down.
   *
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  /** Run sysId on the {@link Arm} */
  public Command sysId() {
    return arm.sysId(
        Volts.of(ClimbConstants.MaximumVoltage),
        Volts.of(ClimbConstants.StepVoltage).per(Second),
        Seconds.of(ClimbConstants.Duration));
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
