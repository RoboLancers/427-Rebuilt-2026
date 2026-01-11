// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // FeedForward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemtry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor Properties to prevent over currenting
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);
  
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel
  .withDiameter(Inches.of(4))
  //Mass of the flywheel
  .withMass(Pounds.of(1))
  //Maximmum speed of the shooter
  .withUpperSoftLimit(RPM.of(1000))
  // Telemetry name and verbosity for the arm
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);

  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  public IntakeSubsystem() {}

  public Command shooterMethodCommand() {
    return runOnce(
      () -> {

      });
  } 

  public boolean shooterCondition() {
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
