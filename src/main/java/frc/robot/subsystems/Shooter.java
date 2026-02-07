package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {
    public Shooter() {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default -> throw new Error();};
    }

    public final ShooterIO io;
    public final ShooterIOInputs inputs = new ShooterIOInputs();

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
    }

    public Command startFlywheels(AngularVelocity velocity) {
        return new RunCommand(() -> {
            io.runFlywheelsVelocity(velocity);
        });
    }

    public Command turnHood(Angle hoodAngle) {
        return new RunCommand(() -> {
            io.rotateHood(hoodAngle);
        });
    }

    public Command startKicker() {
        return new RunCommand(() -> {
            io.runKicker();
        });
    }

    // Runs flywheels and kicker. Command will not end on its own
    public Command startShooting(AngularVelocity velocity, int kickerVoltage) {
        return new ParallelCommandGroup(startFlywheels(velocity), startKicker());
    }

    // Spins up flywheels to speed and turns hood to correct angle. Command will not end on its own
    public Command getReadyToShoot(DoubleSupplier distance) {
        AimValues vals = Constants.Shooter.lookUpTable.get(distance.getAsDouble());

        return new ParallelCommandGroup(startFlywheels(vals.shooterVelocity), turnHood(vals.hoodAngle));
    }

    // Stops robot from shooting
    public Command idle() {
        return new ParallelCommandGroup(
                startShooting(Units.DegreesPerSecond.zero(), 0), turnHood(Units.Degrees.zero()));
    }
}
