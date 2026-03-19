package frc.robot.subsystems.IntakeShooter;

public interface IntakeShooterIOinterface {
    
    @AutoLog
    public static class IntakeShooterIOInputs {
        public int MaxVelocity = (IntakeConstants.MaxVelocity);
        public int StatorAmps = (IntakeConstants.StatorAmps);
        public int StatorCurrentLimit = (IntakeConstants.StatorCurrentLimit);
    }

    default void updateInput(IntakeShooterIOInputs inputs) {}

    default void setTargetVelocity(double RPM) {}

    default void stop() {}
}