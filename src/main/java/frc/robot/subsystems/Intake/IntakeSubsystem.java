// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;

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
  /** Creates a new intake. */
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(IntakeConstants.KP,IntakeConstants.KI, IntakeConstants.KD, DegreesPerSecond.of(IntakeConstants.MaxVelocity), DegreesPerSecondPerSecond.of(IntakeConstants.MaxAcceleration))
  .withSimClosedLoopController(IntakeConstants.KP,IntakeConstants.KI, IntakeConstants.KD, DegreesPerSecond.of(IntakeConstants.MaxVelocity), DegreesPerSecondPerSecond.of(IntakeConstants.MaxAcceleration))
  // FeedForward Constants
  .withFeedforward(new SimpleMotorFeedforward(IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
  .withSimFeedforward(new SimpleMotorFeedforward(IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
  // Telemtry name and verbosity level
  .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft
  .withGearing(IntakeConstants.Intake_GearRatio)
  // Motor Properties to prevent over currenting
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(IntakeConstants.CurrentLimit));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(MotorConstants.Intake_SparkMax_ID, MotorType.kBrushless);
  
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(MotorConstants.numMotors), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel
  .withDiameter(Inches.of(IntakeConstants.FlyWheel_Diameter))
  //Mass of the flywheel
  .withMass(Pounds.of(IntakeConstants.FlyWheel_Mass))
  //Maximmum speed of the intake
  .withUpperSoftLimit(RPM.of(IntakeConstants.SoftLimit))
  // Telemetry name and verbosity for the arm
  .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  public AngularVelocity getVelocity() {return intake.getSpeed();}

  public Command setVelocity(AngularVelocity speed) {return intake.setSpeed(speed);}

  public Command set(double dutyCycle) {return  intake.set(dutyCycle);}

  public IntakeSubsystem() {}

  public Command intakeMethodCommand() {
    return runOnce(
      () -> {

      });
  } 

  public boolean intakeCondition() {
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
