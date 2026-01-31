package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface HopperFloorIO {
    @AutoLog
    public static class HopperFloorIOInputs {
        public AngularVelocity angularVelocity;
    }

    public default void setSpeed(AngularVelocity angularVelocity) {}

    public default void updateInputs(HopperFloorIOInputs hoperFloorInputs) {}

    public default void halt() {}

    public default void runHopper() {}
}
