package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
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
import frc.robot.Constants.ShooterConstants;
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

public class ShooterSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)

  .withClosedLoopController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, DegreesPerSecond.of(ShooterConstants.DegPerSecmagnitude), DegreesPerSecondPerSecond.of(ShooterConstants.DegPerSecPerSecmagnitude))
  .withSimClosedLoopController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, DegreesPerSecond.of(ShooterConstants.DegPerSecmagnitude), DegreesPerSecondPerSecond.of(ShooterConstants.DegPerSecPerSecmagnitude))

  .withFeedforward(new SimpleMotorFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.ka))
  .withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.ka))

  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)

  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  


  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(4))
  .withMass(Pounds.of(1))
  .withUpperSoftLimit(RPM.of(1000))
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel (shooterConfig);
    /**
     * Gets the current velocity of the shooter.
     * 
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {return shooter.getSpeed();}

    public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}
    public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  public ShooterSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
