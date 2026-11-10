package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveTime extends Command {
    public DriveTime(final double time, CommandSwerveDrivetrain drivetrain) {
        this.time = time;
        mDrivetrain = drivetrain;
        this.addRequirements(mDrivetrain);
    }

    public double time;
    private CommandSwerveDrivetrain mDrivetrain;
    private double end;

    @Override
    public void initialize() {
        this.end = Timer.getFPGATimestamp() + this.time;
    }

    @Override
    public void execute() {
        mDrivetrain.controlRobotDrivetrainAutonomus(new ChassisSpeeds(-2, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= this.end;
    }
}
