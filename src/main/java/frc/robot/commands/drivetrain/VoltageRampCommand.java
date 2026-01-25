package frc.robot.commands.drivetrain;

import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class VoltageRampCommand extends Command {
	public VoltageRampCommand(Drivetrain oldDrivetrain, CommandSwerveDrivetrain drivetrain) {
		mOldDrivetrain = oldDrivetrain;
		mDrivetrain = drivetrain;
		this.addRequirements(mDrivetrain);
		VoltageRampCommand.voltage = 0;
	}

	private static double voltage;
	private Drivetrain mOldDrivetrain;
	private CommandSwerveDrivetrain mDrivetrain;

	@Override
	public void initialize() { VoltageRampCommand.voltage = 0; }

	@Override
	public void execute() {
		mOldDrivetrain.runCharacterization(VoltageRampCommand.voltage);
		VoltageRampCommand.voltage += 0.005;
	}

	@Override
	public boolean isFinished() { return VoltageRampCommand.voltage > 0.5; }

	@Override
	public void end(final boolean interrupted) { mDrivetrain.halt(); }

}
