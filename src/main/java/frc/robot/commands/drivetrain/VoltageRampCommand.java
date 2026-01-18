package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class VoltageRampCommand extends Command {
	public VoltageRampCommand(Drivetrain drivetrain) {
		mDrivetrain = drivetrain;
		this.addRequirements(mDrivetrain);
		VoltageRampCommand.voltage = 0;
	}

	private static double voltage;
	private Drivetrain mDrivetrain;

	@Override
	public void initialize() { VoltageRampCommand.voltage = 0; }

	@Override
	public void execute() {
		mDrivetrain.runCharacterization(VoltageRampCommand.voltage);
		VoltageRampCommand.voltage += 0.005;
	}

	@Override
	public boolean isFinished() { return VoltageRampCommand.voltage > 0.5; }

	@Override
	public void end(final boolean interrupted) { mDrivetrain.halt(); }

}
