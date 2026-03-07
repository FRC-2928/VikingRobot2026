package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public AngularVelocity flywheelSpeedA;
        public AngularVelocity flywheelSpeedB;
        public Angle hoodAngle = Units.Radians.zero();
        public AngularVelocity targetFlywheelVelocity = Units.RotationsPerSecond.zero();
        public Angle targetHoodAngle = Units.Degrees.zero();
        public boolean hoodAngleInTolerance = false;
        public boolean flywheelsInTolerance = false;
    }

    public default void rotateHood(Angle hoodAngle) {}

    public default void runFlywheels() {}

    public default void stopFlyWheels() {}

    public default void runFlywheelsVelocity(AngularVelocity speed) {}

    public default void runKicker(Voltage kickerVoltage) {}

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
