package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {
		public Angle position; //may not need to maintain the postion
		public ReverseLimitValue home; //boolean for the home postion of the motor
	}

	public default void set(final double position) {} //set the position of the climb arm

	public default void updateInputs(final ClimberIOInputs inputs) {} //updates the positon value and home boolean

	public default void override(final double dutyCycle) {} //overides the climb if it is interupted

	public default void periodic() {} //runs periodically
}
