package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public class ClimberIOInputs {
		public double position; //may not need to maintain the postion
		public boolean home; //boolean for the home postion of the motor
	}

	public default void set(final double position) {} //set the position of the climb arm

	public default void updateInputs(final ClimberIOInputs inputs) {} //updates the positon value and home boolean

	public default void periodic() {} //runs periodically
}
