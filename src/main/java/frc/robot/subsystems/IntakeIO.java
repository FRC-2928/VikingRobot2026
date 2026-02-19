package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public AngularVelocity angularVelocity = Units.RadiansPerSecond.zero();
        public Distance expansionMotorAngle = Units.Inches.zero();
    }

    public default void setSpeed(double speed) {}

    public default void extend() {}

    public default void retract() {}

    public default void updateInputs(IntakeInputs intakeInputs) {}

    public default void simulationInit() {}

    public default void simPeriodic() {}
}
