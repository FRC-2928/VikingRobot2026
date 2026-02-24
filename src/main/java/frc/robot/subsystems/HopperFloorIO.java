package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface HopperFloorIO {
    @AutoLog
    public static class HopperFloorIOInputs {
        public AngularVelocity angularVelocity;
    }

    public default void setSpeed(AngularVelocity angularVelocity) {}

    public default void updateInputs(HopperFloorIOInputs hoperFloorInputs) {}

    public default void halt() {}

    public default void simPeriodic() {}

    public default void runHopper() {}
}
