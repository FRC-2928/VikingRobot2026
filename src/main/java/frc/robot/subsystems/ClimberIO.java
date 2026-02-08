package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {
		public double position; //may not need to maintain the postion
		public boolean home; //boolean for the home postion of the motor
	}

	public default void set(final double position) {} //set the position of the climb arm

	public default void updateInputs(final ClimberIOInputs inputs) {} //updates the positon value and home boolean

	public default void override(final double dutyCycle) {} //overides the climb if it is interupted

	public default void periodic() {} //runs periodically
}
