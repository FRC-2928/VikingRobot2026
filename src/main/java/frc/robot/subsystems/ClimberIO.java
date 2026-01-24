package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        public double position; // may not need to maintain the postion
        public boolean home;
    }

    public default void set(final double position) {}

    public default void override(final double dutyCycle) {}

    public default void offset(final double offset) {}

    public default void updateInputs(final ClimberIOInputs inputs) {}

    public default void periodic() {}
}
