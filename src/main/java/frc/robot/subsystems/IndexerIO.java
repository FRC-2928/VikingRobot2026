package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public AngularVelocity angularVelocity;
        public AngularVelocity indexerAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         indexerStatorCurrent = Units.Amps.zero();
        public Current         indexerSupplyCurrent = Units.Amps.zero();
    }

    public default void setSpeed(double angularVelocity) {}

    public default void updateInputs(IndexerIOInputs indexerIOInputs) {}

    public default void halt() {}

    public default void simPeriodic() {}

    public default void runIndexer() {}
}
