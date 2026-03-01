package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public AngularVelocity angularVelocity = Units.RadiansPerSecond.zero();
        public Distance expansionMotorAngle = Units.Inches.zero();
        public LinearVelocity expansionAngularVelocity = Units.InchesPerSecond.zero();
    }

    public default void setState(Constants.Intake.IntakeStates state) {}

    public default void extend() {}

    public default void retract() {}

    public default void updateInputs(IntakeInputs intakeInputs) {}

    public default void simulationInit() {}

    public default void simPeriodic() {}

    public default void stateMachinePeriodic() {}

    public default void setWantedState(IntakeIOReal.WantedState state) {}
}
