package frc.robot.subsystems.IntakeShooter;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;


public class IntakeShooter extends SubsystemBase {
  public int FuelCounter = 0;

  protected void execute() {
    SmartDashboard.putNumber("Fuel Number", FuelCounter);
  } 

  /** Creates a new intake. */
  SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
  .withMechanismPosition()
  .withRotorPosition()
  .withMechanismLowerLimit()
  .withMechanismUpperLimit();
  
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD, DegreesPerSecond.of(IntakeConstants.MaxVelocity), DegreesPerSecondPerSecond.of(IntakeConstants.MaxAcceleration))
  .withSimClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD, DegreesPerSecond.of(IntakeConstants.MaxVelocity), DegreesPerSecondPerSecond.of(IntakeConstants.MaxAcceleration))
  // FeedForward Constants
  .withFeedforward(new SimpleMotorFeedforward(IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
  .withSimFeedforward(new SimpleMotorFeedforward(IntakeConstants.ks, IntakeConstants.kv, IntakeConstants.ka))
  // Telemtry name and verbosity level
  .withTelemetry("IntakeMotor", motorTelemetryConfig)
  // Gearing from the motor rotor to final shaft
  .withGearing(IntakeConstants.Intake_GearRatio)
  // Motor Properties to prevent over currenting
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(IntakeConstants.CurrentLimit))
  .withClosedLoopRampRate(Seconds.of(IntakeConstants.ClosedLoopRampRate))
  .withOpenLoopRampRate(Seconds.of(IntakeConstants.OpenLoopRampRate));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(MotorConstants.Intake_SparkMax_ID, MotorType.kBrushless);
  
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(MotorConstants.IntakenumMotors), smcConfig);

  private Debouncer statorDebounce = new Debouncer(IntakeConstants.DebounceTime);

  public boolean isGamePieceIn(){
    return statorDebounce.calculate(sparkSmartMotorController.getStatorCurrent().gte(Amps.of(IntakeConstants.DebounceMagnitude)));
  }

  private  FlyWheelConfig intakeConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(IntakeConstants.FlyWheel_Diameter))
  //Mass of the flywheel
  .withMass(Pounds.of(IntakeConstants.FlyWheel_Mass))
  //Maximmum speed of the intake
  .withUpperSoftLimit(RPM.of(IntakeConstants.SoftLimit))
  // Telemetry name and verbosity for the arm
  .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);

  private FlyWheel intake = new FlyWheel(intakeConfig);

  public IntakeShooter() {}

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
    boolean GamePiece = isGamePieceIn();
    if(GamePiece == true){
      FuelCounter += 1;
    }
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
  public AngularVelocity getVelocity() {return intake.getSpeed();}

  public Command setVelocity(AngularVelocity speed) {return intake.setSpeed(speed);}

  public Command set(double dutyCycle) {return  intake.set(dutyCycle);}
}
