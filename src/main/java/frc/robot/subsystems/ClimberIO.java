package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.commands.climber.climberCommand;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double position; // may not need to maintain the postion
        public boolean home; // boolean for the home postion of the motor
        public climberCommand.ClimberState state;
        public climberCommand.ClimberHeight targetheight;
    }

    public default void updateInputs(final ClimberIOInputs inputs) {} // updates the positon value and home boolean

    public default void override(final double dutyCycle) {} // overides the climb if it is interupted

    public default void periodic() {} // runs periodically

    public default void ascend() {}
}
