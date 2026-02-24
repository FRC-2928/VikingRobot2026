package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveTime extends Command {
    public DriveTime(final double time) {
        this.time = time;
        this.addRequirements(Robot.cont.drivetrain);
    }

    public double time;

    private double end;

    @Override
    public void initialize() {
        this.end = Timer.getFPGATimestamp() + this.time;
    }

    @Override
    public void execute() {
        Robot.cont.drivetrain.control(new ChassisSpeeds(-2, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= this.end;
    }
}
