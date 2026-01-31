package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public AngularVelocity angularVelocity;
    }

    public default void setSpeed(AngularVelocity angularVelocity) {}

    public default void updateInputs(IntakeIOInputs intakeInputs) {}
}
