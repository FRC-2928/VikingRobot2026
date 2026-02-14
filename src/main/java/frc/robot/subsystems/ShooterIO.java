package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public AngularVelocity flywheelSpeedA;
        public AngularVelocity flywheelSpeedB;
        public Angle hoodAngle = Units.Radians.zero();
    }

    public default void rotateHood(Angle hoodAngle) {}

    public default void runFlywheels() {}

    public default void runFlywheelsVelocity(AngularVelocity speed) {}

    public default void runKicker(Voltage kickerVoltage) {}

    public default void updateInputs(final ShooterIOInputs inputs) {}

    public default AngularVelocity getFlywheelVelocity() {
        return null;
    }

    public default Angle getHoodAngle() {
        return null;
    }
}
