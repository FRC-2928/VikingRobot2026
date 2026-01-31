package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public AngularVelocity angularVelocity = Units.RadiansPerSecond.zero();
    }

    public default void setSpeed(double speed) {}

    public default void updateInputs(IntakeInputs intakeInputs) {}
}
