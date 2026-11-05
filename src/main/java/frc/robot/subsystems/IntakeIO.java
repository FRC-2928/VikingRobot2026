package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        // Intake rack
        public Distance intakePosition = Units.Inches.zero();
        public AngularVelocity intakeAngularVelocity = Units.RotationsPerSecond.zero();
        public LinearVelocity intakeRackSpeed = Units.InchesPerSecond.zero();
        public boolean isIntakeHomed = false;
        public boolean isIntakeMaxed = false;
        public Current intakeStatorCurrent = Units.Amps.zero();
        public Current intakeSupplyCurrent = Units.Amps.zero();
        
        // Intake Roller
        public AngularVelocity intakeRollerAngularVelocity = Units.RotationsPerSecond.zero();
        public Current intakeRollerStatorCurrent = Units.Amps.zero();
        public Current intakeRollerSupplyCurrent = Units.Amps.zero();
    }

    public default void setState(Constants.Intake.IntakeStates state) {}

    public default void moveToPosition(final Distance position) {}

    public default void stopMotion() {}

    public default void extendForward(boolean ignoreSoftLimits) {}

    public default void retract(boolean ignoreSoftLimits) {}

    public default void updateInputs(IntakeInputs intakeInputs) {}

    public default void simulationInit() {}

    public default void simPeriodic() {}

}
