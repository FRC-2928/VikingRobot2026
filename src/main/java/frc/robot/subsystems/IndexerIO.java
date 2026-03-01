package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public AngularVelocity angularVelocity;
    }

    public default void setSpeed(double angularVelocity) {}

    public default void updateInputs(IndexerIOInputs indexerIOInputs) {}

    public default void halt() {}

    public default void simPeriodic() {}

    public default void runIndexer() {}
}
