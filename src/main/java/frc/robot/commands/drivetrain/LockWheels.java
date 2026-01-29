package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class LockWheels extends Command {
	private final Drivetrain mDrivetrain;
	private final DriverOI mDriverOI;

	public LockWheels(Drivetrain drivetrain, DriverOI driverOI) {
		mDrivetrain = drivetrain;
		mDriverOI = driverOI;
		addRequirements(mDrivetrain);
	}

	@Override
	public void execute() {
		mDrivetrain.control(Drivetrain.State.locked());
		if(mDrivetrain != null) {
			mDriverOI.getHID().setRumble(RumbleType.kBothRumble, 0.25);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		mDrivetrain.control(Drivetrain.State.forward());
		if(mDriverOI != null) {
			mDriverOI.getHID().setRumble(RumbleType.kBothRumble, 0);
		}
	}
}
