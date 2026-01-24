package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VoltageRampCommand extends Command {
    public VoltageRampCommand() {
        this.addRequirements(Robot.cont.drivetrain);
        VoltageRampCommand.voltage = 0;
    }

    private static double voltage;

    @Override
    public void initialize() {
        VoltageRampCommand.voltage = 0;
    }

    @Override
    public void execute() {
        Robot.cont.drivetrain.runCharacterization(VoltageRampCommand.voltage);
        VoltageRampCommand.voltage += 0.005;
    }

    @Override
    public boolean isFinished() {
        return VoltageRampCommand.voltage > 0.5;
    }

    @Override
    public void end(final boolean interrupted) {
        Robot.cont.drivetrain.halt();
    }
}
