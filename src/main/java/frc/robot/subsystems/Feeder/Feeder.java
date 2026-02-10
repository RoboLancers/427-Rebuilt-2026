package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class Feeder extends SubsystemBase {

  // public int FuelCounter;
  @AutoLog
  public static class FeederInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
    }

  private final FeederInputsAutoLogged feederInputs = new FeederInputsAutoLogged();

  SmartMotorControllerTelemetryConfig motorTelemetryConfig =
      new SmartMotorControllerTelemetryConfig()
          .withMechanismPosition()
          .withRotorPosition()
          .withMechanismLowerLimit()
          .withMechanismUpperLimit();

  SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              FeederConstants.ClosedLoopControllerkP,
              FeederConstants.ClosedLoopControllerkI,
              FeederConstants.ClosedLoopControllerkI,
              DegreesPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSec),
              DegreesPerSecondPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSecPerSec))
          .withSoftLimit(
              Degrees.of(FeederConstants.SoftLimitDegree),
              Degrees.of(FeederConstants.SoftLimitDegreeMagnitude))
          .withGearing(FeederConstants.GearingreductionStages)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("FeederMotor", motorTelemetryConfig);

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              FeederConstants.kP,
              FeederConstants.kI,
              FeederConstants.kD)
          .withSimClosedLoopController(
              FeederConstants.kP,
              FeederConstants.kI,
              FeederConstants.kD)
          .withFeedforward(
              new SimpleMotorFeedforward(
                  FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(GearBox.fromReductionStages(FeederConstants.reductionStages)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(FeederConstants.StatorLimit))
          .withClosedLoopRampRate(Seconds.of(FeederConstants.ClosedLoopRampRate))
          .withOpenLoopRampRate(Seconds.of(FeederConstants.OpenLoopRampRate));

  private SparkMax spark = new SparkMax(FeederConstants.FeederdeviceId, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(FeederConstants.FeedernumMotors), smcConfig);

  // private Debouncer statorDebounce = new Debouncer(FeederConstants.debouncerTime);

  // public boolean isGamePieceIn() {
  //   return
  // statorDebounce.calculate(sparkSmartMotorController.getStatorCurrent().gte(Amps.of(FeederConstants.StatorAmps)));
  // }

  private final FlyWheelConfig FeederConfig =
      new FlyWheelConfig(sparkSmartMotorController)
          .withDiameter(Inches.of(FeederConstants.Diameter))
          .withMass(Pounds.of(FeederConstants.Mass))
          .withUpperSoftLimit(RPM.of(FeederConstants.UpperSoftLimit))
          .withTelemetry("FeederMech", TelemetryVerbosity.HIGH);

  private FlyWheel Feeder = new FlyWheel(FeederConfig);

  /**
   * Gets the current velocity of the Feeder.
   *
   * @return Feeder velocity.
   */

private void updateInputs() {
    feederInputs.velocity = Feeder.getSpeed();
    feederInputs.velocity =  sparkSmartMotorController.getMechanismSetpointVelocity().orElse(RPM.of(0));
    feederInputs.volts = sparkSmartMotorController.getVoltage();
    feederInputs.current = sparkSmartMotorController.getStatorCurrent();
}  

  public AngularVelocity getVelocity() {
    return Feeder.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Feeder/Setpoint", speed);
    return Feeder.setSpeed(speed);
  }

  public Command set(double dutyCycle) {
    Logger.recordOutput("Feeder/DutyCycle", dutyCycle);
    return Feeder.set(dutyCycle);
  }

  public Feeder() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    Logger.processInputs("Feeder", feederInputs);
    Feeder.updateTelemetry();

    // boolean Fuel = isGamePieceIn();
    // if (Fuel) {
    //   FuelCounter -= 1;
    // }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Feeder.simIterate();

  }
}
