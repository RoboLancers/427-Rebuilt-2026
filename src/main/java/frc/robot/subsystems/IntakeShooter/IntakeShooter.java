package frc.robot.subsystems.IntakeShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.OperatorConstants.IsSwerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeAlphaConstants;
import frc.robot.Constants.IntakeConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeShooter extends SubsystemBase {
  public static int FuelCounter = 0;
  public static double ShootSpeed;
  private SparkMax spark = new SparkMax(IntakeConstants.Intake_SparkMax_ID, MotorType.kBrushless);
  //private SparkMax sparkFollower = new SparkMax(IntakeConstants.IntakeFollower_SparkMax_ID, MotorType.kBrushless);

  
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(IntakeAlphaConstants.KP, IntakeAlphaConstants.KI, IntakeAlphaConstants.KD)
          .withSimClosedLoopController(IntakeAlphaConstants.KP, IntakeAlphaConstants.KI, IntakeAlphaConstants.KD)
          // FeedForward Constants
          .withFeedforward(
              new SimpleMotorFeedforward(
                  IntakeAlphaConstants.ks, IntakeAlphaConstants.kv, IntakeAlphaConstants.ka))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  IntakeAlphaConstants.ks, IntakeAlphaConstants.kv, IntakeAlphaConstants.ka))
          // Telemtry name and verbosity level
          .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft
          .withGearing(IntakeAlphaConstants.Intake_GearRatio)
          // Motor Properties to prevent over currenting
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(IntakeAlphaConstants.CurrentLimit))
          .withClosedLoopRampRate(Seconds.of(IntakeAlphaConstants.ClosedLoopRampRate))
          .withOpenLoopRampRate(Seconds.of(IntakeAlphaConstants.OpenLoopRampRate));

  public IntakeShooter() {
    SmartDashboard.putNumber("ShooterSpeed", ShootSpeed);
    // if(IsSwerve) {
    // smcConfig =
    //   new SmartMotorControllerConfig(this)
    //       .withControlMode(ControlMode.CLOSED_LOOP)
    //       // Feedback Constants (PID Constants)
    //       .withClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD)
    //       .withSimClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD)
    //       // FeedForward Constants
    //       .withFeedforward(
    //           new SimpleMotorFeedforward(
    //               IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
    //       .withSimFeedforward(
    //           new SimpleMotorFeedforward(
    //               IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
    //       // Telemtry name and verbosity level
    //       .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
    //       // Gearing from the motor rotor to final shaft
    //       .withGearing(IntakeConstants.Intake_GearRatio)
    //       // Motor Properties to prevent over currenting
    //       .withMotorInverted(false)
    //       .withIdleMode(MotorMode.BRAKE)
    //       .withStatorCurrentLimit(Amps.of(IntakeConstants.CurrentLimit))
    //       .withClosedLoopRampRate(Seconds.of(IntakeConstants.ClosedLoopRampRate))
    //       .withOpenLoopRampRate(Seconds.of(IntakeConstants.OpenLoopRampRate));
    //       //.withFollowers(Pair.of(sparkFollower, true));
    // }else{
    //       smcConfig =
    //   new SmartMotorControllerConfig(this)
    //       .withControlMode(ControlMode.CLOSED_LOOP)
    //       // Feedback Constants (PID Constants)
    //       .withClosedLoopController(IntakeAlphaConstants.KP, IntakeAlphaConstants.KI, IntakeAlphaConstants.KD)
    //       .withSimClosedLoopController(IntakeAlphaConstants.KP, IntakeAlphaConstants.KI, IntakeAlphaConstants.KD)
    //       // FeedForward Constants
    //       .withFeedforward(
    //           new SimpleMotorFeedforward(
    //               IntakeAlphaConstants.ks, IntakeAlphaConstants.kv, IntakeAlphaConstants.ka))
    //       .withSimFeedforward(
    //           new SimpleMotorFeedforward(
    //               IntakeAlphaConstants.ks, IntakeAlphaConstants.kv, IntakeAlphaConstants.ka))
    //       // Telemtry name and verbosity level
    //       .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
    //       // Gearing from the motor rotor to final shaft
    //       .withGearing(IntakeAlphaConstants.Intake_GearRatio)
    //       // Motor Properties to prevent over currenting
    //       .withMotorInverted(false)
    //       .withIdleMode(MotorMode.BRAKE)
    //       .withStatorCurrentLimit(Amps.of(IntakeAlphaConstants.CurrentLimit))
    //       .withClosedLoopRampRate(Seconds.of(IntakeAlphaConstants.ClosedLoopRampRate))
    //       .withOpenLoopRampRate(Seconds.of(IntakeAlphaConstants.OpenLoopRampRate));
    // }

  }

  /** Creates a new intake. */
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(IntakeConstants.IntakenumMotors), smcConfig);

  private Debouncer statorDebounce = new Debouncer(IntakeConstants.DebounceTime);

  public boolean isGamePieceIn() {
    return statorDebounce.calculate(
        sparkSmartMotorController
            .getStatorCurrent()
            .gte(Amps.of(IntakeConstants.DebounceMagnitude)));
  }

  private FlyWheelConfig intakeConfig =
      new FlyWheelConfig(sparkSmartMotorController)
          .withDiameter(Inches.of(IntakeConstants.FlyWheel_Diameter))
          .withMass(Pounds.of(IntakeConstants.FlyWheel_Mass))
          .withUpperSoftLimit(RPM.of(IntakeConstants.SoftLimit))
          .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

protected void execute() {
    SmartDashboard.putNumber("Fuel Number", FuelCounter);
  }
  public Command intakeMethodCommand() {
    return runOnce(() -> {});
  }

  public boolean intakeCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fuel Number", FuelCounter);
    intake.updateTelemetry();
    boolean GamePiece = isGamePieceIn();
    if (GamePiece == true) {
      FuelCounter += 1;
    }
    ShootSpeed = SmartDashboard.getNumber("ShooterSpeed", ShootSpeed);
    SmartDashboard.putNumber("ShooterRPM", ShootSpeed);
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }

  public AngularVelocity getVelocity() {
    return intake.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return intake.setSpeed(speed);
  }

  public Command ManualSpeedControl() {
    return intake.setSpeed(() -> RPM.of(IntakeShooter.ShootSpeed));
  }

  public Command set(double dutyCycle) {
    return intake.set(dutyCycle);
  }
}
