package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Hood data values
        public Angle           hoodAngle = Units.Degrees.zero();
        public Angle           targetHoodAngle = Units.Degrees.zero();
        public AngularVelocity hoodAngularVelocity = Units.RotationsPerSecond.zero();
        public Current         hoodStatorCurrent = Units.Amps.zero();
        public Current         hoodSupplyCurrent = Units.Amps.zero();
        public boolean         hoodAngleInTolerance = false;

        // Shooter A data values
        public AngularVelocity shooterAVelocity;
        // TODO: should really measure for B as well...
        public AngularVelocity targetFlywheelVelocity = Units.RotationsPerSecond.zero();
        public Current         shooterAStatorCurrent = Units.Amps.zero();
        public Current         shooterASupplyCurrent = Units.Amps.zero();
        public boolean         flywheelsInTolerance = false;  // currently only measured for A... should fix this...

        // Shooter B data values
        public AngularVelocity shooterBVelocity = Units.RotationsPerSecond.zero();
        public Current         shooterBStatorCurrent = Units.Amps.zero();
        public Current         shooterBSupplyCurrent = Units.Amps.zero();

        // Kicker data values
        public AngularVelocity kickerVelocity = Units.RotationsPerSecond.zero();
        public Current         kickerStatorCurrent = Units.Amps.zero();
        public Current         kickerSupplyCurrent = Units.Amps.zero();
    }

    public default void rotateHood(Angle hoodAngle) {}

    public default void runFlywheels() {}

    public default void stopFlyWheels() {}

    public default void runFlywheelsVelocity(AngularVelocity speed) {}

    public default void runKicker(AngularVelocity kickerVelocity) {}

    public default void simPeriodic() {}

    public default void updateInputs(final ShooterIOInputs inputs) {}

    public default void nudgeAngleUp() {}

    public default void nudgeAngleDown() {}

    public default void nudgeSpeedUp() {}

    public default void rotateHoodNudge() {}

    public default void runVelocity() {}

    public default void nudgeSpeedDown() {}

    public default void resetNudges() {}

}
