package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveTime extends Command {
	public DriveTime(final double time, Drivetrain drivetrain) {
		this.time = time;
		mDrivetrain = drivetrain;
		this.addRequirements(mDrivetrain);
	}

	public double time;
	private Drivetrain mDrivetrain;
	private double end;

	@Override
	public void initialize() { this.end = Timer.getFPGATimestamp() + this.time; }

	@Override
	public void execute() { mDrivetrain.control(new ChassisSpeeds(-2, 0, 0)); }

	@Override
	public boolean isFinished() { return Timer.getFPGATimestamp() >= this.end; }
}
