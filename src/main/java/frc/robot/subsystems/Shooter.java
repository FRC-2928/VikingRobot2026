package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.AimValues;

public class Shooter extends SubsystemBase {
    public Shooter() {
        this.io = switch (Constants.mode) {
            case REAL -> new ShooterIOReal(this);
            default -> new ShooterIOReal(this);};
    }

    public final ShooterIO io;
    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

    public Command startKicker(Voltage kickerVoltage) {
        return new RunCommand(() -> {
            io.runKicker(kickerVoltage);
        });
    }

    // Runs flywheels and kicker. Command will not end on its own
    public Command startShooting(AngularVelocity velocity, Voltage kickerVoltage) {
        return new ParallelCommandGroup(startFlywheels(velocity), startKicker(kickerVoltage));
    }

    public Command shootWithDistance(Distance distance, Voltage kickerVoltage) {
        return startShooting(getShooterVelocity(distance), kickerVoltage);
    }

    // Spins up flywheels to speed and turns hood to correct angle. Command will not end on its own
    public Command getReadyToShoot(Distance distance) {
        AimValues vals;

        // TODO: pull from the LUT once populated
        vals = new AimValues(Degrees.zero(), Units.DegreesPerSecond.zero());
        // if(Constants.mode == Constants.Mode.REAL) {
        //     vals = Constants.Shooter.lookUpTable.get(distance.in(Units.Meters));
        // }
        // else{
        //     vals = new AimValues(Units.Degrees.of(30), Units.DegreesPerSecond.of(5000));
        // }

        return new ParallelCommandGroup(startFlywheels(vals.shooterVelocity), turnHood(vals.hoodAngle));
    }

    public AngularVelocity getShooterVelocity(Distance distance) {
        if(Constants.mode != Constants.Mode.REAL) {
            return Units.DegreesPerSecond.of(5000);
        }
        return Constants.Shooter.lookUpTable.get(distance.in(Units.Meters)).shooterVelocity;
    }

    // Stops robot from shooting
    public Command idle() {
        return new ParallelCommandGroup(
                startShooting(Units.DegreesPerSecond.zero(), Volts.zero()), turnHood(Units.Degrees.zero()));
    }
}
