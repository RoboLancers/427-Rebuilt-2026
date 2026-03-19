package frc.robot.subsystems.IntakeShooter;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeShooterInputsIO {
    
    @AutoLog
    public static class IntakeShooterIOInputs {
        public int MaxVelocity = (IntakeConstants.MaxVelocity);
        public int MaxAcceleration = (IntakeConstants.MaxAcceleration);
        public int StatorCurrentLimit = (IntakeConstants.CurrentLimit);
    }

    default void updateInput(IntakeShooterIOInputs inputs) {}

    default void setTargetVelocity(double RPM) {}

    default void stop() {}
}