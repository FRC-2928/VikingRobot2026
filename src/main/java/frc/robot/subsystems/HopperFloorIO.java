package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

public interface HopperFloorIO {
    @AutoLog
    public static class HopperFloorIOInputs {
        public AngularVelocity hopperAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         hopperStatorCurrent = Units.Amps.zero();
        public Current         hopperSupplyCurrent = Units.Amps.zero();
    }

    public default void setSpeed(double angularVelocity) {}

    public default void updateInputs(HopperFloorIOInputs hoperFloorInputs) {}

    public default void halt() {}

    public default void simPeriodic() {}

    public default void simulationInit() {}

    public default void runHopper() {}

    public default void runHopperReverse() {}

    //public default void updateInputs(final HopperFloorIOInputs inputs) {}
}
