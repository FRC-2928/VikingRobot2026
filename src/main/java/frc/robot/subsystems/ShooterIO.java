package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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

    public default void runKicker(int kickerVoltage) {}

    public default void updateInputs(final ShooterIOInputs inputs) {}
}
