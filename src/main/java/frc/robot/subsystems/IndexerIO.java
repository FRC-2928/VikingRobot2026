package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public AngularVelocity angularVelocity;
        public AngularVelocity indexerUpAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         indexerUpStatorCurrent = Units.Amps.zero();
        public Current         indexerUpSupplyCurrent = Units.Amps.zero();
        public AngularVelocity indexerDownAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         indexerDownStatorCurrent = Units.Amps.zero();
        public Current         indexerDownSupplyCurrent = Units.Amps.zero();
        public AngularVelocity starWheelsAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         starWheelsStatorCurrent = Units.Amps.zero();
        public Current         starWheelsSupplyCurrent = Units.Amps.zero();

    }

    public default void setSpeedIndexer(double angularVelocity) {}

    public default void setSpeedStarWheels(double dutyCycle) {}

    public default void updateInputs(IndexerIOInputs indexerIOInputs) {}

    public default void halt() {}

    public default void simPeriodic() {}

    public default void runIndexer() {}

    public default void runStarWheels() {}
}
