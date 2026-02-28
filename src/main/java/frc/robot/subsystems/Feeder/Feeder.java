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
import frc.robot.Constants.IntakeConstants;
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
  public static double FeedSpeed;

  // SmartMotorControllerConfig motorConfig =
  //     new SmartMotorControllerConfig(this)
  //         .withClosedLoopController(
  //             FeederConstants.ClosedLoopControllerkP,
  //             FeederConstants.ClosedLoopControllerkI,
  //             FeederConstants.ClosedLoopControllerkI,
  //             DegreesPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSec),
  //             DegreesPerSecondPerSecond.of(FeederConstants.ClosedLoopControllerDegreesPerSecPerSec))
  //         .withSoftLimit(
  //             Degrees.of(FeederConstants.SoftLimitDegree),
  //             Degrees.of(FeederConstants.SoftLimitDegreeMagnitude))
  //         .withGearing(FeederConstants.GearingreductionStages)
  //         .withIdleMode(MotorMode.BRAKE)
  //         .withTelemetry("FeederMotor", motorTelemetryConfig);

  // private SmartMotorControllerConfig smcConfig =
  //     new SmartMotorControllerConfig(this)
  //         .withControlMode(ControlMode.CLOSED_LOOP)
  //         .withClosedLoopController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD)
  //         .withSimClosedLoopController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD)
  //         .withFeedforward(
  //             new SimpleMotorFeedforward(
  //                 FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
  //         .withSimFeedforward(
  //             new SimpleMotorFeedforward(
  //                 FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
  //         .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
  //         .withGearing(FeederConstants.reductionStages)
  //         .withMotorInverted(false)
  //         .withIdleMode(MotorMode.BRAKE)
  //         .withStatorCurrentLimit(Amps.of(FeederConstants.StatorLimit))
  //         .withClosedLoopRampRate(Seconds.of(FeederConstants.ClosedLoopRampRate))
  //         .withOpenLoopRampRate(Seconds.of(FeederConstants.OpenLoopRampRate));
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD)
          .withSimClosedLoopController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD)
          // FeedForward Constants
          .withFeedforward(
              new SimpleMotorFeedforward(
                  FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  FeederConstants.ks, FeederConstants.kv, FeederConstants.ka))
          // Telemtry name and verbosity level
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft
          .withGearing(FeederConstants.reductionStages)
          // Motor Properties to prevent over currenting
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(FeederConstants.StatorLimit))
          .withClosedLoopRampRate(Seconds.of(FeederConstants.ClosedLoopRampRate))
          .withOpenLoopRampRate(Seconds.of(FeederConstants.OpenLoopRampRate));
  private SparkMax spark = new SparkMax(FeederConstants.FeederdeviceId, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(FeederConstants.FeedernumMotors), smcConfig);

  private FlyWheelConfig FeederConfig =
      new FlyWheelConfig(sparkSmartMotorController)
          .withDiameter(Inches.of(FeederConstants.Diameter))
          .withMass(Pounds.of(FeederConstants.Mass))
          .withUpperSoftLimit(RPM.of(FeederConstants.UpperSoftLimit))
          .withTelemetry("FeederMech", TelemetryVerbosity.HIGH);

  private FlyWheel feeder = new FlyWheel(FeederConfig);

  /**
   * Gets the current velocity of the Feeder.
   *
   * @return Feeder velocity.
   */
  public AngularVelocity getVelocity() {
    return feeder.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return feeder.setSpeed(speed);
  }

  public Command set(double dutyCycle) {
    return feeder.set(dutyCycle);
  }

  public Command ManualSpeedControl() {
    return feeder.setSpeed(() -> RPM.of(Feeder.FeedSpeed));
  }

  public Feeder() {
    SmartDashboard.putNumber("FeederSpeed", FeedSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feeder.updateTelemetry();
    FeedSpeed = SmartDashboard.getNumber("FeederSpeed", FeedSpeed);
    SmartDashboard.putNumber("FeederRPM", FeedSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    feeder.simIterate();
  }


}

