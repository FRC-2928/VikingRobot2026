package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockWheels extends Command {
	private final CommandSwerveDrivetrain mDrivetrain;
	private final DriverOI mDriverOI;

	public LockWheels(CommandSwerveDrivetrain drivetrain, DriverOI driverOI) {
		mDrivetrain = drivetrain;
		mDriverOI = driverOI;
		addRequirements(mDrivetrain);
	}

	@Override
	public void execute() {
		mDrivetrain.halt();
		if(mDrivetrain != null) {
			mDriverOI.getHID().setRumble(RumbleType.kBothRumble, 0.25);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		// mDrivetrain.control(Drivetrain.State.forward());
		if(mDriverOI != null) {
			mDriverOI.getHID().setRumble(RumbleType.kBothRumble, 0);
		}
	}
}
