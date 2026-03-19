package frc.robot.subsystems.IntakeShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.IntakeConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeShooterIO implements IntakeShooterInputsIO {
    private final IntakeShooter intakeshooter;
    private final SmartMotorController motorcontrollers;

    public IntakeShooterIO(SubsystemBase IntakeShooter) {

        private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD)
          .withSimClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD)
          // FeedForward Constants
          .withFeedforward(
              new SimpleMotorFeedforward(
                  IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
          // Telemtry name and verbosity level
          .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft
          .withGearing(IntakeConstants.Intake_GearRatio)
          // Motor Properties to prevent over currenting
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(IntakeConstants.CurrentLimit))
          .withClosedLoopRampRate(Seconds.of(IntakeConstants.ClosedLoopRampRate))
          .withOpenLoopRampRate(Seconds.of(IntakeConstants.OpenLoopRampRate))
          .withFollowers(Pair.of(sparkFollower, true));

          private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(IntakeConstants.IntakenumMotors), smcConfig);
    }
}