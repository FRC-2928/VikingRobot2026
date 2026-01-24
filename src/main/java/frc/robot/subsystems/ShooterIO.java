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
        public AngularVelocity uptakeSpeed;
    }

    public default void rotateHood() {}

    public default void runFlywheels() {}

    public default void runFlywheelsVelocity() {}

    public default void runUptake() {}

    public default void updateInputs(final ShooterIOInputs inputs) {}
}
