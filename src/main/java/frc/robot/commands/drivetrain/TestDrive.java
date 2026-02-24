package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TestDrive extends Command {
    public TestDrive() {
        this.addRequirements(Robot.cont.drivetrain);
    }

    @Override
    public void execute() {
        Robot.cont.drivetrain.control(new ChassisSpeeds(2, 0, 0));
    }

    @Override
    public void end(final boolean interrupted) {
        Robot.cont.drivetrain.halt();
    }
}
