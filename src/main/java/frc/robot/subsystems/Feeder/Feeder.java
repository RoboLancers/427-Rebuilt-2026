package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public Feeder() {
    SmartDashboard.putNumber("FeederKP", FeederConstants.kP);
    SmartDashboard.putNumber("FeederKI", FeederConstants.kI);
    SmartDashboard.putNumber("FeederKD", FeederConstants.kD);
    SmartDashboard.putNumber("Feederks", FeederConstants.ks);
    SmartDashboard.putNumber("Feederkv", FeederConstants.kv);
    SmartDashboard.putNumber("Feederka", FeederConstants.ka);

    SmartDashboard.putNumber("FeederUpperSoftLimit", FeederConstants.UpperSoftLimit);
    SmartDashboard.putNumber("FeederCurrentLimit", FeederConstants.StatorLimit);
    SmartDashboard.putNumber("FeederMaxVelocity", FeederConstants.MaxVelocity);
    SmartDashboard.putNumber("FeederMaxAcceleration", FeederConstants.MaxAcceleration);
  }

  // public int FuelCounter;

  SmartMotorControllerTelemetryConfig motorTelemetryConfig =
      new SmartMotorControllerTelemetryConfig()
          .withMechanismPosition()
          .withRotorPosition()
          .withMechanismLowerLimit()
          .withMechanismUpperLimit();

  SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              SmartDashboard.getNumber("FeederKP", FeederConstants.ClosedLoopControllerkP),
              SmartDashboard.getNumber("FeederKI", FeederConstants.ClosedLoopControllerkI),
              SmartDashboard.getNumber("FeederKI", FeederConstants.ClosedLoopControllerkI),
              DegreesPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSec),
              DegreesPerSecondPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSecPerSec))
          .withSoftLimit(
              Degrees.of(
                  SmartDashboard.getNumber(
                      "FeederUpperSoftLimit", FeederConstants.SoftLimitDegree)),
              Degrees.of(FeederConstants.SoftLimitDegreeMagnitude))
          .withGearing(FeederConstants.GearingreductionStages)
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("FeederMotor", motorTelemetryConfig);

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              SmartDashboard.getNumber("FeederKP", FeederConstants.kP),
              SmartDashboard.getNumber("FeederKI", FeederConstants.kI),
              SmartDashboard.getNumber("FeederKD", FeederConstants.kD),
              DegreesPerSecond.of(
                  SmartDashboard.getNumber("FeederMaxVelocity", FeederConstants.MaxVelocity)),
              DegreesPerSecondPerSecond.of(
                  SmartDashboard.getNumber(
                      "FeederMaxAcceleration", FeederConstants.MaxAcceleration)))
          .withSimClosedLoopController(
              SmartDashboard.getNumber("FeederKP", FeederConstants.kP),
              SmartDashboard.getNumber("FeederKI", FeederConstants.kI),
              SmartDashboard.getNumber("FeederKD", FeederConstants.kD),
              DegreesPerSecond.of(
                  SmartDashboard.getNumber("FeederMaxVelocity", FeederConstants.MaxVelocity)),
              DegreesPerSecondPerSecond.of(
                  SmartDashboard.getNumber(
                      "FeederMaxAcceleration", FeederConstants.MaxAcceleration)))
          .withFeedforward(
              new SimpleMotorFeedforward(
                  SmartDashboard.getNumber("Feederks", FeederConstants.ks),
                  SmartDashboard.getNumber("Feederkv", FeederConstants.kv),
                  SmartDashboard.getNumber("Feederka", FeederConstants.ka)))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  SmartDashboard.getNumber("Feederks", FeederConstants.ks),
                  SmartDashboard.getNumber("Feederkv", FeederConstants.kv),
                  SmartDashboard.getNumber("Feederka", FeederConstants.ka)))
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(GearBox.fromReductionStages(FeederConstants.reductionStages)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(
              Amps.of(SmartDashboard.getNumber("FeederCurrentLimit", FeederConstants.StatorLimit)))
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
          .withUpperSoftLimit(
              RPM.of(
                  SmartDashboard.getNumber("FeederUpperSoftLimit", FeederConstants.UpperSoftLimit)))
          .withTelemetry("FeederMech", TelemetryVerbosity.HIGH);

  private FlyWheel Feeder = new FlyWheel(FeederConfig);

  /**
   * Gets the current velocity of the Feeder.
   *
   * @return Feeder velocity.
   */
  public AngularVelocity getVelocity() {
    return Feeder.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return Feeder.setSpeed(speed);
  }

  public Command set(double dutyCycle) {
    return Feeder.set(dutyCycle);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
