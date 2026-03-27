package frc.robot.subsystems.IntakeShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

@Logged
public class IntakeShooter extends SubsystemBase {
  // public static in
  public static int FuelCounter = 0;
  public static double ShootSpeed;
  private SparkFlex spark = new SparkFlex(IntakeConstants.Intake_SparkMax_ID, MotorType.kBrushless);
  private SparkFlex sparkFollower =
      new SparkFlex(IntakeConstants.IntakeFollower_SparkMax_ID, MotorType.kBrushless);

  protected void execute() {
    // SmartDashboard.putNumber("Fuel Number", FuelCounter);
  }

  public IntakeShooter() {
    SmartDashboard.putNumber("ShooterSpeed", ShootSpeed);
  }

  /** Creates a new intake. */
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

  // private Debouncer statorDebounce = new Debouncer(IntakeConstants.DebounceTime);

  // public boolean isGamePieceIn() {
  //   return statorDebounce.calculate(
  //       sparkSmartMotorController
  //           .getStatorCurrent()
  //           .gte(Amps.of(IntakeConstants.DebounceMagnitude)));
  // }

  private FlyWheelConfig intakeConfig =
      new FlyWheelConfig(sparkSmartMotorController)
          .withDiameter(Inches.of(IntakeConstants.FlyWheel_Diameter))
          .withMass(Pounds.of(IntakeConstants.FlyWheel_Mass))
          .withUpperSoftLimit(RPM.of(IntakeConstants.SoftLimit))
          .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  public Command intakeMethodCommand() {
    return runOnce(() -> {});
  }

  public boolean intakeCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Fuel Number", FuelCounter);
    intake.updateTelemetry();
    // boolean GamePiece = isGamePieceIn();
    // if (GamePiece == true) {
    //   FuelCounter += 1;
    // }
    ShootSpeed = SmartDashboard.getNumber("ShooterSpeed", ShootSpeed);
    // SmartDashboard.putNumber("ShooterRPM", ShootSpeed);
    // SmartDashboard.putBoolean("AtCloseSpeed", IsClose());
    // SmartDashboard.putBoolean("AtFarSpeed", IsFar());
    // SmartDashboard.putBoolean("IsShooterRunning", IsShooterRunning());
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }

  public AngularVelocity getVelocity() {
    AngularVelocity velocity = intake.getSpeed();
    return velocity;
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

  public boolean IsClose() {
    //  return intake.isNear(RPM.of(FuelConstants.SpinUpIntakeClose), RPM.of(100)).getAsBoolean();
    if (intake.getSpeed().in(RPM) <= FuelConstants.SpinUpIntakeClose) {
      return true;
    } else {
      return false;
    }
  }

  public boolean IsFar() {
    // return intake.isNear(RPM.of(FuelConstants.SpinUpIntakeFar), RPM.of(100)).getAsBoolean();
    if (intake.getSpeed().in(RPM) <= FuelConstants.SpinUpIntakeFar) {
      return true;
    } else {
      return false;
    }
  }

  public boolean IsShooterRunning() {
    if (intake.getSpeed().in(RPM) > 10) {
      return true;
    }
    return false;
  }
}
