package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Tuning;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends Command {

    private Shooter shooter;

    public SpinUpShooter() {
        shooter = new Shooter();
        this.addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        AngularVelocity velocity = Units.DegreesPerSecond.of(Tuning.flywheelSpeed.get());
        Logger.recordOutput("Shooter/FlywheelSpeedRPS", velocity);
        shooter.io.runFlywheelsVelocity(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
