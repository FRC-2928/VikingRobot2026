package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double height; // may not need to maintain the postion
        public boolean home; // boolean for the home postion of the motor
    }

    public default void updateInputs(ClimberIOInputs climberIOInputs) {}

    public default void override() {} // overides the climb if it is interupted

    public default void periodic() {} // runs periodically 

    public default void goUp() {}

    public default void goDown() {}

    public default void goToPosition(Distance distance) {} 

    public default boolean isEngaged() {} //checks for current spikes to see if the climber has hooked on
}
