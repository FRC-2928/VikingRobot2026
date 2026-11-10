package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VoltageRampCommand extends Command {
    private double voltage;
    private CommandSwerveDrivetrain mDrivetrain;

    public VoltageRampCommand(CommandSwerveDrivetrain drivetrain) {
        mDrivetrain = drivetrain;
        this.addRequirements(mDrivetrain);
        this.voltage = 0;
    }

    @Override
    public void initialize() {
        this.voltage = 0;
    }

    @Override
    public void execute() {
        mDrivetrain.runCharacterization(this.voltage);
        this.voltage += 0.005;
    }

    @Override
    public boolean isFinished() {
        return this.voltage > 0.5;
    }

    @Override
    public void end(final boolean interrupted) {
        mDrivetrain.halt();
    }
}
