package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;

public interface HopperFloorIO {
    @AutoLog
    public static class HopperFloorIOInputs {
        public AngularVelocity hopperAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         hopperStatorCurrent = Units.Amps.zero();
        public Current         hopperSupplyCurrent = Units.Amps.zero();
    }

    public default void setSpeed(double angularVelocity) {}

    public default void setVoltage(double voltage) {}

    public default void updateInputs(HopperFloorIOInputs hoperFloorInputs) {}

    public default void simPeriodic() {}

    public default void simulationInit() {}
    //TODO: remove this method when we remove command usage
    public default void halt() {}

    public default BaseStatusSignal[] getStatusSignals() { return new BaseStatusSignal[0]; }
}
